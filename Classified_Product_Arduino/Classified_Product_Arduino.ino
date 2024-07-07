#include <Servo.h>
#include <AccelStepper.h>

#define Number_Servo 6
int servopin[] = {2, 3, 4, 5, 6, 7}; // Chân kết nối cho các servo
Servo myservo[Number_Servo]; // Mảng các đối tượng Servo

int delayStep = 25; // Thời gian chờ giữa các bước di chuyển
int dochia = 20.0; // Số bước di chuyển
int Wait = 500;    // thời gian chờ vật

// Driver: FULL STEP, Motor 1.8' => 200 bước = 1 vòng 360'
#define Step_pin 8
#define Dir_pin 9
#define Swich_pin 10
#define Home_pin 11
#define Start_pin 12
#define CB1_pin 13
int step, cb1;
AccelStepper Step(1, Step_pin, Dir_pin); //1 là chế độ dùng Driver

float VT0[Number_Servo] = {62, 120, 90, 90, 120, 0}; // Vị trí ban đầu của servo
float VT[Number_Servo];
float ABC[Number_Servo];
//float Toa_do[5];
float ToaDo[5];

int Data = 0; // Dữ liệu từ cổng serial

//- Thông số bảng D-H
float d1 = 135;
float a2 = 195;
float a3 = 115;
float a4 = 110;
float x,y,z;
float theta[6] = {0,0,0,0,0,0};
//float theta_old[6] = {0,0,0,0,0,0};
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < Number_Servo; i++) {
    myservo[i].attach(servopin[i]);
    myservo[i].write(VT0[i]);
    VT[i] = VT0[i];
  }
  Step.setMaxSpeed(800);  //Đặt tốc độ tối đa, mặc định là rất chậm
      //Khi được điều khiển bởi vị trí được cài đặt, bước sẽ tăng tốc để di chuyển ở tốc độ tối đa NÀY và giảm tốc khi nó đến đích.
      //Tốc độ >1000 bước/s là không đáng tin cậy
      //Mặc đinh trong thư viện là 1000 bước/s
  Step.setAcceleration(800);
  pinMode(Swich_pin, INPUT);
  pinMode(Home_pin, INPUT_PULLUP);
  pinMode(Start_pin, INPUT_PULLUP);
  while(digitalRead(Swich_pin) == HIGH) 
  {
    Step.setSpeed(-800); //Cho motor chạy với tốc độ 200 bước/s
    Step.runSpeed();
    //Serial.println(digitalRead(Swich_pin));
  }
  Step.setCurrentPosition(0); //Set vị trí hiện tại của động cơ là 0
  delay(250);
  step = 200;
  while(Step.currentPosition() != step) 
      {
        if(step - Step.currentPosition() >= 0) Step.setSpeed(250); 
        else Step.setSpeed(-250);
        Step.runSpeed();
      }
  step = 0;
  while(Step.currentPosition() != step) 
      {
        if(step - Step.currentPosition() >= 0) Step.setSpeed(250); 
        else Step.setSpeed(-250);
        Step.runSpeed();
      }
  
  // Serial.print("Tốc độ: "); Serial.print(Step.speed());
  // Serial.print("    Vị trí: "); Serial.println(Step.currentPosition());
  delay(2000); //dừng chờ 1s
}

void loop() {
  if(digitalRead(CB1_pin) == LOW)
  {
     cb1 = 1;
     
  }
  if(digitalRead(CB1_pin) == HIGH )
  {
    if(cb1 != 0)
    {
      cb1 = 0;
      Serial.print("1");
      delay(300);
    }

  }
  if(digitalRead(Start_pin) == LOW)
  {
    // Bước 1
  //float ToaDo[5] = {130, 2100, 280, 120, 0};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
  }
  if(digitalRead(Home_pin) == LOW)
  {
    //{62, 120, 90, 90, 120, 0}
    VT[0] = 62;
    VT[1] = 120;
    VT[2] = 90;
    VT[3] = 90;
    VT[4] = 120;
    VT[5] = 0;
    RunServo(VT0, VT);
    step = 0;
    while(Step.currentPosition() != step) 
        {
          if(step - Step.currentPosition() >= 0) Step.setSpeed(500); 
          else Step.setSpeed(-500);
          Step.runSpeed();
        }
  }
  //Serial.println(cb1);
//Serial.println(digitalRead(CB1_pin));
//Serial.println(digitalRead(CB1_pin));
  if (Serial.available()) {
    Data = (Serial.read() - '0'); // Đọc dữ liệu từ cổng serial
    //Serial.println(Data);
    delay(1500);
  }
  switch (Data) 
  {
    case 1:
      Vat1();
      break;
    case 2:
      Vat2();
      break;
  }
  Data = 0;
  // if(digitalRead(Home_pin) == HIGH)
  // {
  //   while(digitalRead(Swich_pin) == HIGH) 
  //   {
  //     Step.setSpeed(-250); //Cho motor chạy với tốc độ 200 bước/s
  //     Step.runSpeed();
  //   }
  //   Step.setCurrentPosition(0); //Set vị trí hiện tại của động cơ là 0
  //   Serial.print("Tốc độ: "); Serial.print(Step.speed());
  //   Serial.print("    Vị trí: "); Serial.println(Step.currentPosition());
  //   delay(1000); //dừng chờ 1s
  //  }

}

void RunServo(float VTC[], float VTM[]) {
  for (int i = 0; i < dochia; i++) {
    for (int j = 0; j < (Number_Servo ); j++) {
      ABC[j] = (float(VTM[j] - VTC[j]) / dochia);
      float x = VTC[j] + ABC[j] * (i+1) ; // Tính toán vị trí mới của servo
      myservo[j].write(x); // Ghi vị trí mới vào servo
      
      //Serial.print(x);
      //Serial.print(" | ");
    }
    delay(delayStep);
    //Serial.println("");
  }
  // Lưu vị trí cuối cùng của servo vào VTC
  for (int i = 0; i < Number_Servo; i++) {
    VTC[i] = VTM[i];
  }
}

void InverseKinematics(float Px, float Py, float Pz, float O)
{
  float th1=atan2(Py,Px);
  float Nx = Px*cos(th1) + Py*sin(th1);
  float Ny = Pz - d1 + a4;
  float th3= -acos((sq(Nx) + sq(Ny) - sq(a2*1.0) - sq(a3*1.0))/(2.0*a2*a3));
  float th2= atan2((Ny*(a2 + a3*cos(th3)) - a3*sin(th3)*Nx) , (Nx*(a2 + a3*cos(th3)) + a3*sin(th3)*Ny));
  float th4= th2 + th3 + 3.14/2 ;
    theta[1] =  degrees(th1);
    theta[2] =  degrees(th2);
    theta[3] = -degrees(th3);
    theta[4] =  degrees(th4);
    theta[5] = theta[1] - O;
    // Serial.println(theta[1]);
    // Serial.println(theta[2]);
    // Serial.println(theta[3]);
    // Serial.println(theta[4]);
}
void Run(float Toa_do[])
{
    // Vị trí 1
    //Toa_do[] = {130, 2200, 280, 120, 0};
    InverseKinematics(Toa_do[0], 0, Toa_do[2], 0);
    VT[0] = 180 - theta[2];
    VT[1] = theta[2];
    VT[2] = 200 - theta[3];
    VT[3] = theta[4];
    VT[4] = Toa_do[3];
    VT[5] = Toa_do[4];
    step = Toa_do[1];
    while(Step.currentPosition() != step) 
        {
          if(step - Step.currentPosition() >= 0) Step.setSpeed(500); 
          else Step.setSpeed(-500);
          Step.runSpeed();
        }
    RunServo(VT0, VT);
    
}

void Vat1()
{
  // Bước 1
  //float ToaDo[5] = {130, 2100, 280, 120, 0};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
  // Bước 2
  //float ToaDo[5] = {130, 2100, 240, 120, 0};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 240;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
  delay(Wait);
  // Bước 3
  //float ToaDo[5] = {130, 2100, 240, 120, 40};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 240;
  ToaDo[3] = 20;
  ToaDo[4] = 30;
  Run(ToaDo);
  // Bước 4
  //float ToaDo[] = {130, 2100, 280, 120, 40};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 30;
  Run(ToaDo);
  // Bước 5
  //float ToaDo = {110, 100, 200, 120, 40};
  ToaDo[0] = 110;
  ToaDo[1] = 50;
  ToaDo[2] = 150;
  ToaDo[3] = 120;
  ToaDo[4] = 30;
  Run(ToaDo);
  // Bước 6
  //float ToaDo = {110, 100, 200, 120, 0};
  ToaDo[0] = 110;
  ToaDo[1] = 50;
  ToaDo[2] = 150;
  ToaDo[3] = 120;
  ToaDo[4] = 0;
  Run(ToaDo);
  delay(300);
  // Bước 7
  //float ToaDo = {110, 100, 280, 120, 0};
  ToaDo[0] = 110;
  ToaDo[1] = 50;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
  //Serial.println("HOAN THANH :v");
  // Bước 1
  //float ToaDo[5] = {130, 2100, 280, 120, 0};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
}

void Vat2()
{
  // Bước 1
  //float ToaDo[5] = {130, 2100, 280, 120, 0};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
  // Bước 2
  //float ToaDo[5] = {130, 2100, 240, 120, 0};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 240;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
  delay(Wait);
  // Bước 3
  //float ToaDo[5] = {130, 2100, 240, 120, 40};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 240;
  ToaDo[3] = 20;
  ToaDo[4] = 30;
  Run(ToaDo);
  // Bước 4
  //float ToaDo[] = {130, 2100, 280, 120, 40};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 30;
  Run(ToaDo);
  // Bước 5
  //float ToaDo = {110, 300, 200, 120, 40};
  ToaDo[0] = 110;
  ToaDo[1] = 600;
  ToaDo[2] = 150;
  ToaDo[3] = 120;
  ToaDo[4] = 30;
  Run(ToaDo);
  // Bước 6
  //float ToaDo = {110, 300, 200, 120, 0};
  ToaDo[0] = 110;
  ToaDo[1] = 600;
  ToaDo[2] = 150;
  ToaDo[3] = 120;
  ToaDo[4] = 0;
  Run(ToaDo);
  delay(300);
  // Bước 7
  //float ToaDo = {110, 300, 280, 120, 0};
  ToaDo[0] = 110;
  ToaDo[1] = 600;
  ToaDo[2] = 280;
  ToaDo[3] = 120;
  ToaDo[4] = 0;
  Run(ToaDo);
  //Serial.println("HOAN THANH :v");
  // Bước 1
  //float ToaDo[5] = {130, 2100, 280, 120, 0};
  ToaDo[0] = 150;
  ToaDo[1] = 2100;
  ToaDo[2] = 280;
  ToaDo[3] = 20;
  ToaDo[4] = 0;
  Run(ToaDo);
}