#define F_CPU 16000000
#define U_BAUD_RATE(BAUD_RATE) ((float)(F_CPU*64/(16*(float)BAUD_RATE))+0.5)
//baud rate 계산

#define TWI_BAUD(F_SCL,T_rise) (((float)F_CPU/(float)F_SCL)-10-((float)F_CPU*T_rise/100000))/2
#define pi 3.141592
#define RADIANS_TO_DEGREES 180/pi
#define fs 131.0
#define MPU_ADDR 0x68

volatile uint8_t*PORTa_PINCTLR_SDA=0x0412;//PA2 pullup
volatile uint8_t*PORTa_PINCTLR_SCL=0x0413;//PA3 pullup
volatile uint8_t*PORTa_DIR=0x0400;//PA2 out
volatile uint8_t*TWI_CTRLA=0x08a0;//sdasetup,sdahold
volatile uint8_t*TWI_MCTRLA=0x08a3;//enable/timeout
volatile uint8_t*TWI_MCTRLB=0x08a4;//flush/ackact/master command
volatile uint8_t*TWI_MSTATUS=0x08a5;//
volatile uint8_t*TWI_MBAUD=0x08a6;//baud rate
volatile uint8_t*TWI_MADDR=0x08a7;//slave addr
volatile uint8_t*TWI_MDATA=0x08a8;//data

uint16_t raw_accel_x,raw_accel_y,raw_accel_z,raw_gyro_x,raw_gyro_y,raw_gyro_z;//센서에서 얻은 raw data
volatile float base_accel_x,base_accel_y,base_accel_z,base_gyro_x,base_gyro_y,base_gyro_z;//센서 바이어스 값
volatile float accel_x1,accel_y1,accel_z1;//베이스 뺀 가속도
volatile float gyro_x,gyro_y;//자이로 각속도
volatile float accel_x,accel_y;// 최종 가속도 값
volatile float angle_gyro_x=0,angle_gyro_y=0;//최종 각도
volatile float angle_x=0,angle_y=0,angle_z=0;//상보필터 거친 각도
volatile float alpha=0.96;
volatile float x_mv,y_mv,z_mv;

volatile long i;
//uint8_t MPU_addr=0x68;//mpu-6050의 I2C slave address

volatile uint8_t* PORTc_DIR=0x0440;//tx 핀을 아웃풋,rx는 인풋
//usart핀 direction 제어
volatile uint8_t* U_RXDATAL=0x0820;//rx 데이터 읽기
volatile uint8_t* U_TXDATAL=0x0822;//tx 데이터 내보내기
volatile uint8_t* U_STATUS=0x0824;//상태 확인하기
volatile uint8_t* U_CTRLB=0x0826;//Tx,Rx enable
volatile uint8_t* U_CTRLC=0x0827;//통신상태 초기화
volatile uint16_t* U_BAUDL=0x0828;
volatile uint16_t* U_BAUDH=0x0829;//baud rate
//usart1 변수

volatile uint8_t* PORTf_DIR=0x04a0;//tx 핀을 아웃풋,rx는 인풋: PF0,PF1
//usart핀 direction 제어
volatile uint8_t* U2_RXDATAL=0x0840;//rx 데이터 읽기
volatile uint8_t* U2_TXDATAL=0x0842;//tx 데이터 내보내기
volatile uint8_t* U2_STATUS=0x0844;//상태 확인하기
volatile uint8_t* U2_CTRLB=0x0846;//Tx,Rx enable
volatile uint8_t* U2_CTRLC=0x0847;//통신상태 초기화
volatile uint16_t* U2_BAUDL=0x0848;
volatile uint16_t* U2_BAUDH=0x0849;//baud rate
//usart2 변수

volatile uint8_t*PORTe_DIR_button=0x0480;//PE DIR에 접근
volatile uint8_t*PORTe_IN_button=0x0488;//PE IN에 접근
volatile uint8_t*PORTe_PINCTRL_button1=0x0490;//PE0 PINCRTL에 접근
volatile uint8_t*PORTe_PINCTRL_button2=0x0491;//PE1 PINCRTL에 접근
volatile uint8_t*PORTe_PINCTRL_button3=0x0492;//PE2 PINCRTL에 접근
int push1,push2,push3;
//button1: 펜 굵기 변경 button2: 펜 색상 변경 button3: 지우개/펜 모드

volatile uint8_t* PORTd_PINCTRL=0x0470;//PD0 pincontrol
volatile uint8_t* ADC_CTRLA=0x0600;//resolution/free run/enable설정
volatile uint8_t* ADC_CTRLB=0x0601;//accumulation num설정(평균을 내어 노이즈 제거)
volatile uint8_t* ADC_CTRLC=0x0602;//reference voltage selection/clk prescale
volatile uint8_t* ADC_CTRLD=0x0603;//시작 딜레이/샘플링 딜레이
volatile uint8_t* ADC_MUXPOS=0x0606;//채널선택
volatile uint8_t* ADC_STCONV=0x0608;//start conversion
volatile uint8_t* ADC_INTFLAGS=0x060B;//result ready
volatile uint8_t* ADC_RESL=0x0610;
volatile uint8_t* ADC_RESH=0x0611;//result
//adc 변수 

volatile uint8_t *TCA_CTRLA=0x0a00;//prescaling/enable
volatile uint8_t *TCA_CTRLESET=0x0a05;//
volatile uint16_t *TCA_PERL=0x0a26;//top value 
volatile uint16_t *TCA_PERH=0x0a27;
volatile uint8_t *TCA_CNTL=0x0a20;
volatile uint8_t *TCA_CNTH=0x0a21;//cnt value
//TCA 변수

void MPU_getsensor();

void init_TWI();
void init_MPU();
void recover_TWI();
uint8_t start_TWI(uint8_t device_ADDR);
uint8_t wait_ACK_TWI();
void repeat_start_TWI(uint8_t device_ADDR);
uint8_t read_TWI(uint8_t *data,uint8_t ack_flag);
uint8_t write_TWI(uint8_t *data);
void stop_TWI();
uint8_t read_bytes_TWI(uint8_t slave_addr, uint8_t *addr_ptr,uint8_t slave_reg,uint8_t num_byte);
uint8_t write_bytes_TWI(uint8_t slave_addr, uint8_t *addr_ptr, uint8_t slave_reg, uint8_t num_byte);
void calibrate_sensor();//센서의 초가값 조정
  

/*void init_TWI();
uint8_t start_TWI(uint8_t);
uint8_t wait_ACK();
void repeat_start_TWI(uint8_t);
uint8_t read_TWI(uint8_t*,uint8_t);
uint8_t write_TWI(uint8_t*);
void stop_TWI(); 
uint8_t read_byte_TWI(uint8_t,uint8_t*,uint8_t,uint8_t );//slave는 byte array로 구성 ->master의 array로 바꿔주기
uint8_t write_byte_TWI(uint8_t,uint8_t*,uint8_t,uint8_t);
*/
void init_button();
void init_tc();
void tc_delay(volatile double );
void init_uart();
void init_uart2();
char read_data();
void send_data(char );
void print_string(char*);
void mode_change();//버튼으로 모드 변경하기
void init_adc();
void itoa(uint16_t , char* );

//함수 선언

void MPU_getsensor(){
  uint8_t data[14];
  read_bytes_TWI(MPU_ADDR,data,0x3b,14);
  raw_accel_x=data[0]<<8 | data[1];
  raw_accel_y=data[2]<<8 | data[3];
  raw_accel_z=data[4]<<8 | data[5];
  raw_gyro_x=data[8]<<8 | data[9];
  raw_gyro_y=data[10]<<8 | data[11];
  raw_gyro_z=data[12]<<8 | data[13]; 
}

void init_TWI(){
  *PORTa_DIR=0x04;//PA2 out
  //*PORTa_PINCTLR_SDA=0x08;//PA2 pullup
  //*PORTa_PINCTLR_SCL=0x08;//PA3 pullup

  *TWI_MBAUD=(volatile double)TWI_BAUD(100000,0);//보드 레이트 100kHz  
  *TWI_MCTRLB=0x08;//flush->1 :clear internal state of th master
  *TWI_MCTRLA=0x01;//master enable, 나머지는 disable
  *PORTa_DIR|=0x0c;//둘 다 out으로
  
  *TWI_MSTATUS|=0x01;//IDLE 상태
  *TWI_MSTATUS|=(0x80|0x40|0x04|0x01);//RIF/WIF/BUSERR clear
}
void init_MPU(){
  write_bytes_TWI(MPU_ADDR,0,0x6b,1);
}
void recover_TWI(){
  uint8_t j;
  *TWI_MCTRLB|=0x08;//flush
  *TWI_MCTRLA=0;

  *PORTa_DIR&= ~(0x04);//sda
  for(j=0;j<9;j++){
    *PORTa_DIR|=0x08;//scl
    for(i=0;i<5;i++);
    *PORTa_DIR&=~(0x08);//scl
    for(i=0;i<5;i++);
  }
  *TWI_MCTRLB=0x08;//flush
  *TWI_MCTRLA=0x01;//master enable
  *PORTa_DIR=0x04;//sda
  *TWI_MSTATUS|=(0x80|0x40|0x04|0x01);//RIF/WIF/BUSERR/IDLE
  *TWI_MCTRLB=0x08;//flush
  *TWI_MCTRLA=0x01;//master enable
  *PORTa_DIR|=0x08;//scl
  *TWI_MSTATUS|=(0x80|0x40|0x04|0x01);//RIF/WIF/BUSERR/IDLE
}
uint8_t start_TWI(uint8_t device_addr){
  *TWI_MSTATUS|=0xc0;//clear RIF/WIR
  if(*TWI_MSTATUS&0x04) return 4;//buserror
  *TWI_MADDR=device_addr;
  return 0;
}
uint8_t wait_ACK_TWI(){
  while(!((*TWI_MSTATUS&0x80)==0x80)&&!((*TWI_MSTATUS&0x40)==0x40));
  *TWI_MSTATUS|=0xc0;//rif/wif clear
  if(*TWI_MSTATUS&0x04) return 4;//buserror
  if(*TWI_MSTATUS&0x08) return 2;//arbitrary lost
  if(*TWI_MSTATUS&0x10) return 1;//RXACK
  return 0;

}
void repeat_start_TWI(uint8_t device_addr){
  *TWI_MADDR=device_addr;
}
uint8_t read_TWI(uint8_t*data,uint8_t ack_flag){
  if((*TWI_MSTATUS&0x02)==0x02){//TWI가 owner상태일 때
    while(!((*TWI_MSTATUS&0x80)==0x80));
    *TWI_MSTATUS|=0xc0;//rif/wif clear
    if(*TWI_MSTATUS&0x04) return 4;//buserror
    if(*TWI_MSTATUS&0x08) return 2;//arbitrary lost
    if(*TWI_MSTATUS&0x10) return 1;//RXACK
    if(ack_flag==0) *TWI_MCTRLB&=~0x04;//ACKACT 0
    else *TWI_MCTRLB|=0x04;
    
    *data=*TWI_MDATA;
    if(ack_flag==0){
      *TWI_MCTRLB|=0x02;//MCMD_RECVTRANS
    }
    return 0;
  }
  else return 8;
}
uint8_t write_TWI(uint8_t*data){
   if((*TWI_MSTATUS&0x02)==0x02){//owner일때
    *TWI_MDATA=*data;
    while(!((*TWI_MSTATUS&0x40)==0x40));
    if(*TWI_MSTATUS&0x04) return 4;//buserror
    if(*TWI_MSTATUS&0x10) return 1;//RXACK
    return 0;
   }
   else return 8;
}
void stop_TWI(){
  *TWI_MCTRLB|=0x03;//stop
}
uint8_t read_bytes_TWI(uint8_t slave_addr,uint8_t*addr_ptr,uint8_t slave_reg,uint8_t num_byte){
  uint8_t stat;
  stat=start_TWI(slave_addr<<1);//lsb는 0
  if(stat!=0) goto error;
  stat=wait_ACK_TWI();
  if(stat==1){
    stop_TWI();
    return 1;
  }
  if(stat!=0) goto error;
  stat=write_TWI(&slave_reg);
  if(stat!=0) goto error;
  repeat_start_TWI((slave_addr<<1)+1);
  while(num_byte>1){
    stat=read_TWI(addr_ptr,0);
    if(stat!=0) goto error;
    addr_ptr++;
    num_byte--;
  }
  read_TWI(addr_ptr,1);
  if(stat!=0) goto error;
  stop_TWI();
  return 0;
  error:
  recover_TWI();
  return 0xff;  
}
uint8_t write_bytes_TWI(uint8_t slave_addr,uint8_t*addr_ptr,uint8_t slave_reg,uint8_t num_byte){
  uint8_t stat;
  stat=start_TWI(slave_addr<<1);
  if(stat!=0) goto error;
  stat=wait_ACK_TWI();
  if(stat==1){
    stop_TWI();
    return 1;
  }
  if(stat!=0) goto error;
  stat=write_TWI(&slave_reg);
  if(stat!=0) goto error;
  while(num_byte>0){
    stat=write_TWI(addr_ptr);
    if(stat!=0) goto error;
    addr_ptr++;
    num_byte--;
  }
  stop_TWI();
  return 0;
  error:
  recover_TWI();
  return 0xff;
}
void init_button(){
  *PORTe_DIR_button=0x00;//PE port를 모두 input으로 사용
  *PORTe_PINCTRL_button1=0x08;//PE0 pullup enable
  *PORTe_PINCTRL_button2=0x08;//PE1 pullup enable
  *PORTe_PINCTRL_button3=0x08;//PE2 pullup enable
}
void init_tc(){
  *TCA_PERL=0xfa;
//  *TCA_PERH=0x00;//top value를 250으로
  *TCA_CTRLA=0x5b;//ftca-250kHz/enable
}

void tc_delay(volatile double s){
  volatile double i,value;
  for(i=0;i<s;i++){ 
    init_tc();
    value=0;
    while(249>value){
      value=*TCA_CNTL;
    }
    *TCA_CTRLA=0x00;//tc 끄기
  }
}
void init_uart(){ 
  *PORTc_DIR=0x01;//tx는 아웃풋, rx는 인풋
  *U_CTRLB|=0xd0;//Tx,Rx enable
  *U_CTRLC=0x03;//parity x, stopbit 1,char size 8bit

  *U_BAUDL=(uint16_t)U_BAUD_RATE(9600); //보드레이트 설정
}
void init_uart2(){ 
  *PORTf_DIR=0x01;//tx는 아웃풋, rx는 인풋
  *U2_CTRLB|=0xd0;//Tx,Rx enable
  *U2_CTRLC=0x03;//parity x, stopbit 1,char size 8bit

  *U2_BAUDL=(uint16_t)U_BAUD_RATE(9600); //보드레이트 설정
}
char read_data(){
  while((*U2_STATUS&0x80)!=0x80){
    tc_delay(5);
    }//rx데이터가 들어올 때 까지 대기
  return *U2_RXDATAL;
}

void send_data(char data){   
  while(*U2_STATUS&0x20!=0x20){;} 
  *U2_TXDATAL=data; 
  tc_delay(3);//문자를 출력하기 위한 시간
}
void print_string(char*str){
  while(*str!=0x00){
    send_data(*str);
    str++;
  }
}
void mode_change(){
  if(((*PORTe_IN_button)&0x01)!=0x01){
    push1++;
    tc_delay(200);
    
    if((push1%3)==0){
      print_string("S\n");
    }
    else if((push1%3)==1){
      print_string("M\n");
    }
    else if((push1%3)==2){
      print_string("L\n");
    }
  }//button 1
    
 if(((*PORTe_IN_button)&0x02)!=0x02){
    push2++;
    tc_delay(200);
    
    if((push2%3)==0){
      print_string("BLACK\n");
    }
    else if((push2%3)==1){
      print_string("RED\n");
    }
    else if((push2%3)==2){
      print_string("BLUE\n");
    }
  }//button 2
  if(((*PORTe_IN_button)&0x04)!=0x04){
    push3++;
    tc_delay(200);
    
    if((push3%2)==0){
      print_string("WRITE\n");
    }
    else if((push3%2)==1){
      print_string("ERASE\n");
    }
  }//button 3
}
void init_adc(){
  *ADC_CTRLA=0x02;//resolution/freerun
  *ADC_CTRLB=0x00;//accumulation x
  *ADC_CTRLC=0x06;//internal ref vol/125kHZ
  *ADC_MUXPOS=0x00;//0번 아날로그 값

  *PORTd_PINCTRL|=0x04;//digital input disable
  
  *ADC_CTRLA|=0x01;//enable adc
  *ADC_STCONV=0x01; //start conversion
}
void itoa(uint16_t num, char* str){
  volatile int radix=10;//10진수
  volatile int deg=1;
  volatile int cnt=0;
  volatile int i=0;

  while(1){//자릿수 뽑기
    if((num/deg)>0) cnt++;
    else{
      deg/=radix;
      break;
    }   
    deg*=radix;   
  }
  if(num==0){//0이 들어왔을 경우
    *str='0'; 
    *(str+1)='\0';//스트링 마지막에 널 문자 
  }
  else{//0을 제외한 다른 수가 들어온 경우
    for(i=0;i<cnt;i++){
    *(str+i)=num/deg+'0';
    num-=((num/deg)*deg);
    deg/=radix;//자리수 줄임 
    }
   *(str+i) = '\0';//스트링 마지막에 널 문자
  }
}
char* ftoa(volatile float number)
{

    char* first;
    char string[50];
    char string_2[50];
    double  number_2,change;
    int  fractional,decimal;


    decimal = (int) number;     //extracting decimal part form fractional

    number_2 = (double) decimal;

    change = number - number_2;

    fractional = change*1000; //extracting fractional part and changing it into integer

    // itoa(decimal,first,10);
    // itoa(fractional,sec,10);

    sprintf(string,"%d",decimal);           //changing both parts into string
    sprintf(string_2,"%d",fractional);

    strcat(string,".");                     //adding dot between numbers
    strcat(string,string_2);
    first = string;

    return first;                           //returning final string
}

volatile int mousePress(unsigned int result){
  volatile int mouse_press=0;
  if(result>=186){
    mouse_press=1;//압력센서 값이 150보다 크면 1
  }
  else{
    mouse_press=0;//압력센서 값이 150보다 작으면 0
  }
  return mouse_press;
}

void calibrate_sensor(){
  volatile int num_readings=10;
  volatile float accel_x=0;
  volatile float accel_y=0;
  volatile float accel_z=0;
  volatile float gyro_x=0;
  volatile float gyro_y=0;
  volatile float gyro_z=0;

  MPU_getsensor();//첫 번째 데이터는 폐기
  for(volatile int i=0;i<num_readings;i++){
    MPU_getsensor();
    accel_x+=raw_accel_x;
    accel_y+=raw_accel_y;
    accel_z+=raw_accel_z;
    gyro_x+=raw_gyro_x;
    gyro_y+=raw_gyro_y;
    gyro_z+=raw_gyro_z;
    tc_delay(50);
  }
  accel_x/=num_readings;
  accel_y/=num_readings;
  accel_z/=num_readings;
  gyro_x/=num_readings;
  gyro_y/=num_readings;
  gyro_z/=num_readings;

  base_accel_x=accel_x;
  base_accel_y=accel_y;
  base_accel_z=accel_z;
  base_gyro_x=gyro_x;
  base_gyro_y=gyro_y;
  base_gyro_z=gyro_z; 
}

void complimentary_filter(){
  volatile float dt=0.001;
  //가속도값 아크탄젠트->각도 변화
  accel_x1=raw_accel_x-base_accel_x;
  accel_y1=raw_accel_y-base_accel_y;
  accel_z1=raw_accel_z-base_accel_z;
  
  accel_x=atan(accel_y1/sqrt(pow(accel_x1,2)+pow(accel_z1,2)))*RADIANS_TO_DEGREES;
  accel_y=atan(accel_x1/sqrt(pow(accel_y1,2)+pow(accel_z1,2)))*RADIANS_TO_DEGREES;

  //자이로 -32766~+32766을 실제 250 degree/s로 변환
  gyro_x=(raw_gyro_x-base_gyro_x)/fs;
  gyro_y=(raw_gyro_y-base_gyro_y)/fs;

  //변화량을 적분
  angle_gyro_x=angle_x+dt*gyro_x;
  angle_gyro_y=angle_y+dt*gyro_y;

  //상보필터
  angle_x=alpha*angle_gyro_x+(1.0-alpha)*accel_x;
  angle_y=alpha*angle_gyro_y+(1.0-alpha)*accel_y;

}
void move_cursor(volatile float prev_x,volatile float now_x,volatile float prev_y,volatile float now_y){
  if(now_x-prev_x>0.3){
    print_string("UP\n");
  }
  else if(prev_x-now_x>0.3){
    print_string("DOWN\n");
  }
  if(now_y-prev_y>0.3){
    print_string("RIGHT\n");
  }
  else if(prev_y-now_y>0.3){
    print_string("LEFT\n");
  }
}

void setup(){
 volatile float sensitivity=3;
 volatile float prev_x=0,now_x=0,prev_y=0,now_y=0;
 init_uart();
 init_uart2();
 init_TWI();
 init_MPU();
 calibrate_sensor();

 init_adc();
 init_button();
 push1=0;push2=0;push3=0;//버튼 1,2,3을 각각 누른 횟수
 unsigned int result_h=0,result_l=0,result=0;
 char value[10];//itoa사용할 문자열 담을 배열
 volatile int prev=0,now=0;
 

 while(1){
  tc_delay(3);
  MPU_getsensor();
  complimentary_filter();
  
  x_mv=angle_x/sensitivity/1.1*(-1);
  y_mv=angle_y/sensitivity/1.1*(-1);

   prev_x=now_x;
   now_x=x_mv;
   prev_y=now_y;
   now_y=y_mv;
   move_cursor(prev_x,now_x,prev_y,now_y);
  
  while((*ADC_INTFLAGS&0x01)!=0x01){;}//a2d가 진행 될 때 까지 대기
  result_h=*ADC_RESH;//[9:8]
  result_l=*ADC_RESL;//[7:0]결과
  result=result_l+(result_h*256);//하나의 변수로
  //result에 ADC의 결과 담기

  prev=now;
  now=mousePress(result);
  if(now>prev){//0->1이 될때 set
    print_string("1\n");
  }
  else if(now<prev){//1->0이 될 때 reset
    print_string("0\n");
  }//마우스 press/release 신호 보내기
  
  mode_change();//버튼 눌림 제어
  

 } 

}

void loop(){}
