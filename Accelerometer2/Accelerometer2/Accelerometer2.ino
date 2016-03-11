#define pinX A2
#define pinY A1
#define pinZ A0

double gradientX = 0.006329169889;
double gradientY = 0.006270058854;
double gradientZ = 0.006165677356;
double interceptX = -2.076832094;
double interceptY = -2.175366909;
double interceptZ = -1.585254887;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Time (ms)\tX\tY\tZ");
}

void loop() {
  // put your main code here, to run repeatedly:
  //double RG = gradientR*analogRead(A0) + interceptR;
  double X = gradientX*analogRead(pinX) + interceptX;
  double Y = gradientY*analogRead(pinY) + interceptY;
  double Z = gradientZ*analogRead(pinZ) + interceptZ;
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(X);
  Serial.print("\t");
  Serial.print(Y);
  Serial.print("\t");
  Serial.println(Z);
}
