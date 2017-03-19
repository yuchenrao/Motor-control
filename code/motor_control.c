#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"// include other header files here
#include <stdio.h>
#define BUF_SIZE 200
#define VOLTS_PER_COUNT (3.3/1024)
#define CORE_TICK_TIME 25    // nanoseconds between core ticks
#define SAMPLE_TIME 10       // 10 core timer ticks = 250 ns
#define DELAY_TICKS 20000000 // delay 1/2 sec, 20 M core ticks, between messages
#define NUMSAMPS 100   // number of points in waveform
#define PLOTPTS 100
#define NUM 2000
static volatile int Waveform[NUMSAMPS];   // waveform
static volatile int length;
static volatile int Trajectory[NUM];
static volatile int Reference_current;
static volatile float Reference_tra_current;
static volatile int ADCarray[PLOTPTS];
static volatile float Tra_ADCarray[NUM];
static volatile int REFarray[PLOTPTS];
static volatile float Tra_REFarray[NUM];
static volatile int StoringData = 0;
static volatile float Kpi = 0, Kii = 0, Kpp = 0, Kip = 0, Kdp = 0;
static volatile int Eint = 0, angle, eprev = 0;

void makeWaveform();
int cal_cur();
unsigned int adc_sample_convert(int pin);
void set_mode(int num);

int percent;
enum mode_t mode;
enum mode_t {
    IDLE,
    PWM,
    ITEST,
    HOLD,
    TRACK};

void __ISR(_TIMER_2_VECTOR, IPL4SOFT) Controller(void) { // _TIMER_2_VECTOR = 8

  static int counter = 0;
  static int count = 0;
  static int plotind = 0;
  // index for data arrays; counts up to PLOTPTS
  static int adcval = 0;
  //int adcval;
  char buffer[BUF_SIZE];
  float e, u, unew;

  switch (mode) {
    case 0:
    {
      OC1RS = 0;
      break;
    }
    case 1:
    {
      OC1RS = 4000*abs(percent)/100.0;
      break;
    }
    case 2:
    {
      char buffer[BUF_SIZE];
      adcval = cal_cur();    // sample and convert pin 0
      // read th ADC value
      e = Waveform[counter] - adcval;
      Eint = Eint + e;
      if (Eint > 500){
        Eint = 500;
      }
      else if (Eint < -500){
        Eint = -500;
      }
      u = Kpi*e + Kii*Eint;
      LATDbits.LATD3 = 1;
      if (u < 0){
        u = -u;
        LATDbits.LATD3 = 0;
      }
      if (u>4000){
        u = 4000;
      }
      OC1RS = u;
      if (StoringData) {
        ADCarray[plotind] = adcval;
          // store data in global arrays
        REFarray[plotind] = Waveform[counter];
        plotind++;
        if (plotind == PLOTPTS) {
          // if max number of plot points plot is reached,
          plotind = 0;
          // reset the plot index
          StoringData = 0;
          // tell main data is ready to be sent to MATLAB
        }
        }
      counter++;
      // add one to counter every time ISR is entered
      if (counter == NUMSAMPS) {
        counter = 0; // rollover counter over when end of waveform reached
        set_mode(0);
        Eint = 0;
        e = 0;
      }
      break;
    }
    case 3:
    {
      adcval = cal_cur();    // sample and convert pin 0
      // read th ADC value
      e = Reference_current - adcval;
      Eint = Eint + e;
      if (Eint > 500){
        Eint = 500;
      }
      else if (Eint < -500){
        Eint = -500;
      }
      u = Kpi*e + Kii*Eint;
      if (u < 0){
        u = 0;
      }
      if (u>4000){
        u = 4000;
      }
      OC1RS = u;
      break;
    }
    case 4:
    {
      adcval = cal_cur();    // sample and convert pin 0
      // read th ADC value
      e = Reference_tra_current - adcval;
      Eint = Eint + e;
      if (Eint > 500){
        Eint = 500;
      }
      else if (Eint < -500){
        Eint = -500;
      }
      u = Kpi*e + Kii*Eint;
      if (u < 0){
        u = 0;
      }
      if (u>4000){
        u = 4000;
      }
      OC1RS = u;
      break;
    }
  }
  IFS0bits.T2IF = 0;
}

void __ISR(_TIMER_4_VECTOR, IPL5SOFT) PosController(void){
  char buffer[BUF_SIZE];
  static int direction;
  static int count = 0;
  static int plotind = 0;
  // direction = LATDbits.LATD6;
  // if (direction == 1){
  //   LATDbits.LATD6 = 0;
  // }
  // else{
  //   LATDbits.LATD6 = 1;
  // }
  switch (mode) {
    case 3:
    {
      float n, m, e, Edot, up;
      m = 448*4.0;
      n = 360.0 * (encoder_counts(1) - 32768.0)/m;
      e = angle - n;
      if (e>=0){
        LATDbits.LATD3 = 0;
      }
      else{
        e = -e;
        LATDbits.LATD3 = 1;
      }
      Eint = Eint + e;
      Edot = e - eprev;
      eprev = e;
      up = Kpp*e+Kip*Eint+Kdp*Edot;
      Reference_current = up;

      break;
    }
    case 4:
   {
      float n, m, e, Edot, ut;
      //int current_angle;
      m = 448*4.0;
      n = 360.0 * (encoder_counts(1) - 32768.0)/m;
      sprintf(buffer,"The PIC controller mode is currently IDLE.\r\n");
      NU32_WriteUART3(buffer);
      e = Trajectory[count]- n;
      if (e>=0){
        LATDbits.LATD3 = 0;
      }
      else{
        e = -e;
        LATDbits.LATD3 = 1;
      }
      Eint = Eint + e;
      Edot = e - eprev;
      eprev = e;
      ut = Kpp*e+Kip*Eint+Kdp*Edot;
      Reference_tra_current = ut;

      if (StoringData) {
        Tra_ADCarray[plotind] = n;
          // store data in global arrays
        Tra_REFarray[plotind] = Trajectory[count];
        plotind++;
        if (plotind == length) {
          // if max number of plot points plot is reached,
          plotind = 0;
          // reset the plot index
          StoringData = 0;
          // tell main data is ready to be sent to MATLAB
        }
        }
      count++;
      // add one to counter every time ISR is entered
      if (count == length) {
        count = 0; // rollover counter over when end of waveform reached
        set_mode(2);
        Eint = 0;
        e = 0;
        Edot = 0;
        }
     }
  }
  IFS0bits.T4IF = 0;
}

void makeWaveform() {
  int i = 0, center = 0, A = 200; // square wave, fill in center value and amplitude
  char buffer[BUF_SIZE];
  for (i = 0; i < NUMSAMPS; ++i) {
    if ( i < 25) {
      Waveform[i] = center + A;
    }
    else if (i < 50){
      Waveform[i] = center - A;
    }
    else if (i < 75){
      Waveform[i] = center + A;
    }
    else {
      Waveform[i] = center - A;
    }
    // sprintf(buffer,"The current is %d mA.\r\n", Waveform[i]);
    // NU32_WriteUART3(buffer);
  }
}

void set_mode(int num){
  mode = num;
}

void get_mode(int num){
  char buffer[BUF_SIZE];
  if (num == 0){
    sprintf(buffer,"The PIC controller mode is currently IDLE.\r\n");
    NU32_WriteUART3(buffer);
  }
  else if (num == 1){
    sprintf(buffer,"The PIC controller mode is currently PWM.\r\n");
    NU32_WriteUART3(buffer);
  }
  else if (num == 2){
    sprintf(buffer,"The PIC controller mode is currently ITEST.\r\n");
    NU32_WriteUART3(buffer);
  }
  else if (num == 3){
    sprintf(buffer,"The PIC controller mode is currently HOLD.\r\n");
    NU32_WriteUART3(buffer);
  }
  else if (num == 4){
    sprintf(buffer,"The PIC controller mode is currently TRACK.\r\n");
    NU32_WriteUART3(buffer);
  }
}

void ADC_init(void){

  AD1CON1bits.ADON = 1;    // turn on A/D converter
  AD1PCFGbits.PCFG0 = 0;  // AN0 is an adc pin
  AD1CON3bits.ADCS = 2;    //ADC clock period is Tad = 2*(ADCS+1)*Tpb =2*3*12.5ns = 75ns
  AD1CON1bits.ON = 1;    // turn on A/D converter
  AD1CON1bits.SSRC = 0b111;  // auto conversion
  AD1CON1bits.ASAM = 0;     //manual sampling
}

unsigned int adc_sample_convert(int pin) { // sample & convert the value on the given

  unsigned int elapsed = 0, finish_time = 0;
  AD1CHSbits.CH0SA = pin;
  // connect chosen pin to MUXA for sampling
  AD1CON1bits.SAMP = 1;
  // start sampling
  elapsed = _CP0_GET_COUNT();
  finish_time = elapsed + SAMPLE_TIME;
  while (_CP0_GET_COUNT() < finish_time) {
  ;
  // sample for more than 250 ns
  }
  AD1CON1bits.SAMP = 0;
  // stop sampling and start converting
  while (!AD1CON1bits.DONE) {
  ;
  // wait for the conversion process to finish
  }
  return ADC1BUF0;
  // read the buffer with the result
}

int cal_cur(void){
  int current;
  current = 1000*(adc_sample_convert(0)*3.3/1023-1.65)/(100*0.015) + 13;
  return current;
}

void periph_init(){

  __builtin_disable_interrupts();
  percent = 25;
  T3CONbits.TCKPS = 0;     // Timer2 prescaler N=4 (1:4)
  PR3 = 3999;              // period = (PR2+1) * N * 12.5 ns = 100 us, 10 kHz
  TMR3 = 0;                // initial TMR2 count is 0
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1CONbits.OCTSEL = 1;
  OC1RS = 1000;              // initialize before turning OC1 on; afterward it is read-only
  OC1R = 1000;

  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.ON = 1;       // turn on OC1

  T2CONbits.TCKPS = 2;     // Timer2 prescaler N=4 (1:4)
  PR2 = 3999;              // period = (PR2+1) * N * 12.5 ns = 100 us, 10 kHz
  TMR2 = 0;                // initial TMR2 count is 0

  T2CONbits.ON = 1;        // turn on Timer2
  TRISDbits.TRISD3 = 0;    // make F3 as a output pin
  LATDbits.LATD3 = 1; //set motor rotation direction

  IPC2bits.T2IP = 4;            // step 4: interrupt priority 2
  IPC2bits.T2IS = 1;            // step 4: interrupt priority 1
  IFS0bits.T2IF = 0;            // step 5: clear the int flag
  IEC0bits.T2IE = 1;            // step 6: enable INT0 by set
  __builtin_enable_interrupts();
}

void pos_con_init(){
  __builtin_disable_interrupts();
  // position ISR init 200HZ
  T4CONbits.TCKPS = 0b101;
  PR4 = 6249;
  TMR4 = 0;
  T4CONbits.ON = 1;
  TRISDbits.TRISD6 = 0;
  LATDbits.LATD6 = 1;

  IPC4bits.T4IP = 5;            // step 4: interrupt priority 2
  IPC4bits.T4IS = 2;            // step 4: interrupt priority 1
  IFS0bits.T4IF = 0;            // step 5: clear the int flag
  IEC0bits.T4IE = 1;            // step 6: enable INT0 by set
  __builtin_enable_interrupts();
}

int main()
{
  char buffer[BUF_SIZE];
  char message[100];
  int i = 0;
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  encoder_init();
  ADC_init();
  periph_init();
  pos_con_init();
  set_mode(0);// set IDEL mode
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;
  makeWaveform();

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      //return the mode
      case 'a':
      {
        sprintf(buffer,"The motor current is %d ADC counts.\r\n", adc_sample_convert(0));
        NU32_WriteUART3(buffer);
        break;
      }
      case'b':
      {
        sprintf(buffer,"The motor current is %d mA\r\n",cal_cur());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'c':
      {
        sprintf(buffer,"The motor angle is %d counts.\r\n", encoder_counts(1));
        NU32_WriteUART3(buffer);
        break;
      }
      case 'd':    // get the angle
      {
        float n, m;
        m = 448*4.0;
        n = 360.0 * (encoder_counts(1) - 32768.0)/m;
        sprintf(buffer,"The motor angle is %f degrees.\r\n", n);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'e':
      {
        sprintf(buffer,"Reseat: %d\r\n", encoder_counts(0));
        NU32_WriteUART3(buffer);
        break;
      }
      case'f':
      {
        // sprintf(buffer,"What PWM valuewould you like [-100 to 100]?: ");
        // NU32_WriteUART3(buffer);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &percent);
        set_mode(1);
        if (percent>=0){
          LATDbits.LATD3 = 1;
          // sprintf(buffer,"PWM has been set to %d in the counterclockwise direction.\r\n", percent);
          // NU32_WriteUART3(buffer);
        }
        else{
          LATDbits.LATD3 = 0;
          // sprintf(buffer,"PWM has been set to %d in the clockwise direction.\r\n", percent);
          // NU32_WriteUART3(buffer);
        }
        break;
      }
      case 'g':
      {
        // sprintf(buffer,"Enter your desired Kp current gain [recommended: 4.76]: \r\n");
        //NU32_WriteUART3(buffer);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &Kpi);

        // sprintf(buffer,"Enter your desired Ki current gain [recommended: 0.32]: \r\n");
        // NU32_WriteUART3(buffer);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &Kii);

        // sprintf(buffer,"Sending Kp=%f and Ki=%f to the current controller.\r\n", Kp, Ki);
        // NU32_WriteUART3(buffer);
        break;
      }
      case 'h':
      {
        sprintf(buffer,"The current controller is using Kp=%f and Ki=%f. \r\n", Kpi, Kii);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'i':
      {
        // sprintf(buffer,"Enter your desired Kp position gain [recommended: 2]: \r\n");
        // NU32_WriteUART3(buffer);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &Kpp);

        // sprintf(buffer,"Enter your desired Ki current gain [recommended: 0.3]: \r\n");
        // NU32_WriteUART3(buffer);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &Kip);

        // sprintf(buffer,"Enter your desired Kd current gain [recommended: 200]: \r\n");
        // NU32_WriteUART3(buffer);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &Kdp);

        // sprintf(buffer,"Sending Kp=%f, Ki=%f and Kd=%f to the current controller.\r\n", Kpp, Kip, Kdp);
        // NU32_WriteUART3(buffer);
        break;
      }
      case 'j':
      {
        sprintf(buffer,"The current controller is using Kp=%f, Ki = %f and Kd=%f. \r\n", Kpp, Kip, Kdp);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'k':
      {
        set_mode(2);
        StoringData = 1;
        // message to ISR to start storing data
        while (StoringData) {
        // wait until ISR says data storing is done
        ; // do nothing
        }
        sprintf(buffer, "%d\r\n", PLOTPTS);
        NU32_WriteUART3(buffer);
        for (i=0; i<PLOTPTS; i++) {
          // send plot data to MATLAB
          // when first number sent = 1, MATLAB knows we’re done
          sprintf(buffer, "%d %d\r\n", REFarray[i], ADCarray[i]);
          NU32_WriteUART3(buffer);
      }
        break;
      }
      case 'l':
      {
        // sprintf(buffer, "Go to the angle.\r\n");
        // NU32_WriteUART3(buffer);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &angle);
        set_mode(3);
        break;
      }
      case 'm':
      {
        int i = 0;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &length);
        for (i = 0; i < length; ++i){
          NU32_ReadUART3(buffer,BUF_SIZE);
          sscanf(buffer, "%f", &Trajectory[i]);
        }
        break;
      }
      case 'n':
      {
        int i = 0;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &length);
        sprintf(buffer, "The length is %d\r\n", length);
        NU32_WriteUART3(buffer);
        for (i = 0; i < length; ++i){
          NU32_ReadUART3(buffer,BUF_SIZE);
          sscanf(buffer, "%f", &Trajectory[i]);
          sprintf(buffer, "The number is %f\r\n", Trajectory[i]);
        NU32_WriteUART3(buffer);
        }
        break;
      }
      case 'o':
      {
        StoringData = 1;
        set_mode(4);
        // message to ISR to start storing data
        while (StoringData) {
        // wait until ISR says data storing is done
        ; // do nothing
        }
        sprintf(buffer, "%d\r\n", length);
        NU32_WriteUART3(buffer);
        for (i=0; i<length; i++) {
          // send plot data to MATLAB
          // when first number sent = 1, MATLAB knows we’re done
          sprintf(buffer, "%f %f\r\n", Tra_REFarray[i], Tra_ADCarray[i]);
          NU32_WriteUART3(buffer);
      }
        break;
      }
      case 'p':
      {
        set_mode(0);
        // sprintf(buffer,"Set mode to IDLE\r\n");
        // NU32_WriteUART3(buffer);
        break;
      }
      case 'q':
      {
        set_mode(0);
        break;
      }
      case 'r':
      {
        get_mode(mode);
        // if (number == 0){
        //   sprintf(buffer, "The motor current mode is IDLE.\r\n");
        //   NU32_WriteUART3(buffer);
        // }
        // else if (number == 1){
        //   sprintf(buffer, "The motor current mode is PWM.\r\n");
        //   NU32_WriteUART3(buffer);
        // }
        // else if (number == 2){
        //   sprintf(buffer, "The motor current mode is ITEST.\r\n");
        //   NU32_WriteUART3(buffer);
        // }
        // else if (number == 3){
        //   sprintf(buffer, "The motor current mode is HOLD.\r\n");
        //   NU32_WriteUART3(buffer);
        // }
        // else if (number == 4){
        //   sprintf(buffer, "The motor current mode is TRACK.\r\n");
        //   NU32_WriteUART3(buffer);
        // }
        break;
      }
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}



