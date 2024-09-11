class PID {
  private:
    double kp, ki, kd;
    double previousError, integral;
    unsigned long lastTime;
    
  public:
    PID(double kp, double ki, double kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      previousError = 0;
      integral = 0;
      lastTime = millis();
    }

    double calculate(double setPoint, double currentSpeed) {
      unsigned long now = millis();
      double timeChange = (double)(now - lastTime) / 1000.0;
      lastTime = now;

      double error = setPoint - currentSpeed;

      integral += error * timeChange;

      double derivative = (error - previousError) / timeChange;

      double output = kp * error + ki * integral + kd * derivative;
      
      previousError = error;

      return output;
    }
};
double exponentialSmoothing(double input, double previousOutput, double alpha) {
  return alpha * input + (1 - alpha) * previousOutput;
}
const int motorPin = 9;
double targetSpeed = 100;
double currentSpeed = 0;
double alpha = 0.1;
PID pid(1.0, 0.1, 0.05); 

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  double actualSpeed = analogRead(A0); 
  currentSpeed = exponentialSmoothing(targetSpeed, currentSpeed, alpha);
  double controlSignal = pid.calculate(currentSpeed, actualSpeed);
  controlSignal = constrain(controlSignal, 0, 255);
  analogWrite(motorPin, controlSignal);
  Serial.print("Target Speed: ");
  Serial.print(targetSpeed);
  Serial.print(" | Actual Speed: ");
  Serial.print(actualSpeed);
  Serial.print(" | Control Signal: ");
  Serial.println(controlSignal);
  delay(100);
}
