#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire,0.2,0.8);

#define IN1 1
#define IN2 0
#define IN3 2
#define IN4 3
#define ENCA1 4
#define ENCB1 5
#define ENCA2 6
#define ENCB2 7
#define BUTTON_IN 14

int direction;
int pos1, pos2, prevPos1, prevPos2;
double x, y, vx, vy, stx, sty, yaw, encYaw;
double W = 10.85, wheelDiameter = 6.7, dowelDist = 6.0;
double encCount = 715.6, distScale = 1.01, thetaScale = 1.07;

int minLSpeed = 207, minRSpeed = 187, lfss = 207, rfss = 200, lbss = 200, rbss = 207;
int vMax = 58, vMin = 25;

int dtt = 5000, dts = 5000;
long prevT;

double aG;
double initRead = 0;

const int MAX_TOKENS = 1000;
const float MIN_SPEED = 30.0;
const float OFFSET = -4; //If the expected time is 70 and the actual time is 75, then OFFSET = -5
const float TURN_TIME = 0.535;

bool SIM;
const char* CMD_STRING = "81 25 L 90 -90 -100 R 100 L 100 L 50 R 50 R 100 R 40 -88 R 87";
// const char* CMD_STRING = "81 25 L 90 -190 R 100 L 100 L 50 R 50 R 50 L 50 L 44";
// const char* CMD_STRING = "81 25 R 100 L 100 L 100 L 50 R 50 R 50 L 50 L 44";

double PIDVals[1][5] = {{9,0,1000,0,0}};
double ltTol = 8, rtTol = 7, sTol = 0.5;

void setup() {

  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(1000000);
  mpu6050.begin();

  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stop();

  pinMode(BUTTON_IN,INPUT_PULLDOWN);

  while(buttonState()!=1);
  while(buttonState()!=0);

  SIM = true;
  Serial.println("start sim");
  parseAndExecute(CMD_STRING);
  Serial.println("end sim");

  while(buttonState()!=1);
  while(buttonState()!=0);
  Serial.println("start run");

  SIM = false;

  mpu6050.calcGyroOffsets(true, 1000, 500);
  for(int i = 0; i < 3; i++) Serial.println();

  double sum = 0.0;
  for(int i = 0; i < 10; i++) {
    sum += getYaw();
    delay(2);
  }
  initRead = (sum/10.0);

  pos1 = 0; pos2 = 0; prevPos1 = 0; prevPos2 = 0;
  x = 0; y = -dowelDist; vx = 0; vy = 0; yaw = 0; encYaw = 0; aG = 0.98, stx = 0; sty = 0;

  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,CHANGE);

  direction = 0;
  prevT = micros();

  parseAndExecute(CMD_STRING);

  // straight(25);
  // delay(90);
  // turn('L');
  // delay(90);
  // straight(25);

  // for(int i = 0; i < 20; i++) {
  //   straight(50);
  //   delay(100);
  //   turn('L');
  //   delay(100);
  // }

}

void loop() {
  // Serial.println(buttonState());
  // getDists();

  // // Odometry testing
  // updatePos(5000);
  // delay(5);
  // Serial.print(x);
  // Serial.print(" ");
  // Serial.print(y);
  // Serial.print(" ");
  // Serial.print(encYaw);
  // Serial.print(" ");
  // Serial.print(getYaw());
  // Serial.print(" ");
  // Serial.println(yaw);

  // // Gyro testing
  // delay(5);
}

void straightSim(int dist) {
  Serial.print("Moving ");
  Serial.print(dist);
  Serial.println(" cm");
}

void turnSim(char dir) {
  Serial.print("Turning ");
  Serial.println(dir);
} 

void pauseSim(float delayTime) {
  Serial.print("Pausing for ");
  Serial.print(delayTime);
  Serial.println(" seconds");
}

int tokenize(char *str, char *tokens[], int maxTokens) {
  int count = 0;
  char *token = strtok(str, " ");
  while (token != NULL && count < maxTokens) {
    tokens[count++] = token;
    token = strtok(NULL, " ");
  }
  return count;
}

void parseAndExecute(const char *commandStr) {
  char buf[128];
  strncpy(buf, commandStr, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';
  
  char *tokens[MAX_TOKENS];
  int numTokens = tokenize(buf, tokens, MAX_TOKENS);
  
  if (numTokens < 2) {
    Serial.println("Not enough tokens to execute.");
    return;
  }
  
  float totalTime = atof(tokens[0]);
  
  float funcTimes[MAX_TOKENS];
  int distances[MAX_TOKENS];
  
  int numCommands = numTokens - 1;
  
  for (int i = 0; i < numCommands; i++) {
    char *cmd = tokens[i+1];
    if (cmd[0]=='L'||cmd[0]=='R') funcTimes[i] = TURN_TIME;
    else {
      int dist = atoi(cmd);
      distances[i] = dist;
      // Movement time: absolute distance / 30 (cm/sec)
      float moveTime = abs(dist) / MIN_SPEED;
      funcTimes[i] = moveTime;
    }
  }
  
  float totalFuncTime = 0;
  for (int i = 0; i < numCommands; i++) {
    totalFuncTime += funcTimes[i];
  }
  
  int numDelays = numCommands - 1;
  float remainingTime = totalTime - totalFuncTime;
  float delayTime = (numDelays > 0) ? (remainingTime + OFFSET) / numDelays : 0;
  
  for (int i = 0; i < numCommands; i++) {
    char *cmd = tokens[i+1];
    if (cmd[0] == 'L' || cmd[0] == 'R') {
      float execTime = funcTimes[i];
      if (SIM) {
        turnSim(cmd[0]);
      } else {
        turn(cmd[0]);
      }
    } else {
      int dist = atoi(cmd);
      if (SIM) {
        straightSim(dist);
      } else {
        straight(dist);
      }
    }
    if (i < numCommands - 1) {
      if (SIM) {
        pauseSim(delayTime);
      } else {
        delay(delayTime * 1000);
      }
    }
  }
}

double straight(double dist) {
  int sign = ((dist<0)?-1:1);
  if(direction%2==0) sty += ((direction==0)?1:-1)*dist;
  else stx += ((direction==3)?1:-1)*dist;

  long startTime = micros();
  double lSpeed = ((sign==1)?lfss:lbss), rSpeed = ((sign==1)?rfss:rbss);

  setMotor(sign, (int)lSpeed, 1);
  setMotor(sign, (int)rSpeed, 2);
  prevT = micros();

  while (abs(((direction%2==0)?y-sty:x-stx))>sTol) {
    if(micros()-prevT<dts) continue;
    updatePos(micros()-prevT);
    prevT = micros();
    double horizPID = PID(0,((direction%2==0)?stx:sty),((direction%2==0)?x:y));
    setMotor(sign, (int)constrain(lSpeed+((direction==0||direction==1)?1:-1)*horizPID, minLSpeed, 255), 1);
    setMotor(sign, (int)constrain(rSpeed-((direction==0||direction==1)?1:-1)*horizPID, minRSpeed, 255), 2);

  }
  stop();
  long startT = prevT;
  while(micros()-startT<100000) {
    if(micros()-prevT<dtt) continue;
    updatePos(micros()-prevT);
    prevT = micros();
  }

  return (micros()-startTime)/1000000.0;
}

double turn(char dir) {
  int sign = ((dir=='L')?1:-1);
  direction = (direction+sign)%4;
  if(direction<0) direction += 4;

  double target = direction*90.0;
  long startTime = micros();

  setMotor(-sign, minLSpeed-8, 1); //-8
  setMotor(sign, minRSpeed+15, 2); //+15
  
  while(abs(yawError(direction))>((dir=='L')?ltTol:rtTol)) {
    if(micros()-prevT<dtt) continue;
    updatePos(micros()-prevT);
    prevT = micros();
  }
  stop();
  long startT = prevT;
  while(micros()-startT<100000) {
    if(micros()-prevT<dtt) continue;
    updatePos(micros()-prevT);
    prevT = micros();
  }

  return (micros()-startTime)/1000000.0;
}

double yawError(double dir) {
  double error = dir*90-yaw;
  if((dir==0&&yaw>180)||(dir==1&&yaw>270)) error += 360;
  else if(dir==3&&yaw<90) error -= 360;
  return error;
}

double PID(int index, double target, double current) {
  double error = target - current;
  PIDVals[index][4] += error;
  double derivative = error - PIDVals[index][3];
  PIDVals[index][3] = error;
  return (PIDVals[index][0]*error + PIDVals[index][1]*PIDVals[index][4] + PIDVals[index][2]*derivative);
}

void updatePos(long tDiff) {

  double dD1 = distScale*(((double)pos1-prevPos1)/encCount)*PI*wheelDiameter;
  double dD2 = distScale*(((double)pos2-prevPos2)/encCount)*PI*wheelDiameter;
  double dD = (dD1+dD2)/2.0, dTheta = ((dD2-dD1)/W);
  double xDiff = -dD*sin(yaw*PI/180.0+(dTheta/2.0)), yDiff = dD*cos(yaw*PI/180.0+(dTheta/2.0));
  x += xDiff; y += yDiff;
  vx = xDiff/tDiff*1000000.0; vy = yDiff/tDiff*1000000.0;
  encYaw = boundYaw(encYaw + thetaScale*dTheta*180.0/PI);

  double gyroYaw = getYaw();
  if(gyroYaw-encYaw>300.0) encYaw += 360.0;
  else if(encYaw-gyroYaw>300.0) gyroYaw += 360.0;
  yaw = boundYaw(aG*gyroYaw+(1.0-aG)*encYaw);

  prevPos1 = pos1; prevPos2 = pos2;

  double dowelX = x-dowelDist*sin(yaw*PI/180.0), dowelY = y+dowelDist*cos(yaw*PI/180.0);

}

double boundYaw(double raw) {
  raw = fmod(raw,360.0);
  if(raw<0) raw+=360.0;
  return raw;
}

double getYaw() {
  mpu6050.update();
  return boundYaw(mpu6050.getAngleZ()*360.0/357.2225-initRead);
}

void stopMotor(int motor) {
  setMotor(0,0,motor);
}

void stop() {
  stopMotor(1);
  stopMotor(2);
}

void readEncoder1() {
  if (digitalRead(ENCA1)==digitalRead(ENCB1)) pos1--;
  else pos1++;
}

void readEncoder2() {
  if (digitalRead(ENCA2)==digitalRead(ENCB2)) pos2++;
  else pos2--;
}

void setMotor(int dir, int pwmVal, int motor) {
  int in1, in2;
  if(motor==1) {
    in1 = IN1;
    in2 = IN2;
  } else {
    in1 = IN3;
    in2 = IN4;
  }
  if(dir == 1) {
    analogWrite(in1,pwmVal);
    digitalWrite(in2,LOW);
  } else if(dir == -1) {
    analogWrite(in2,pwmVal);
    digitalWrite(in1,LOW);
  } else if(dir == 0) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

int buttonState() {
  if(digitalRead(BUTTON_IN)==HIGH) {
    delay(10);
    if(digitalRead(BUTTON_IN)==HIGH) return 1;
  } else {
    delay(10);
    if(digitalRead(BUTTON_IN)==LOW) return 0;
  }
  return -1;
}