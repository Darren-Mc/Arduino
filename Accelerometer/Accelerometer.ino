#include <Average.h>
//double Xl=258.758,X2=504.3441,X3=741.2438,
//double Y1=283.705,Y2=529.6951,Y3=770.9762;
//double Z1=140.1667,Z2=389.774,Z3=633.8848;
double XYZmin[] = {258.758,283.705,140.1667};
double XYZmid[] = {504.3441,529.6951,389.774};
double XYZmax[] = {741.2438,770.9762,633.8848};    
double gforce[3];
double gradientR = -0.00426;
double gradientH = -0.00405;
double interceptR = 1.891;
double interceptH = 2.14;

Average<int> H(10);
Average<int> R(10);

// the setup routine runs once when you press reset:  
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  analogReference(EXTERNAL);
  Serial.println("Time (ms)\tX\tY\tZ");
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  // print out the value you read:
  //double XYZ[3];
  H.push(analogRead(A0));
  //Serial.print(H.mean());
  Serial.print("\t");
  R.push(analogRead(A1));
  //Serial.println(R.mean());
  //printGforce(XYZ);
  printOrientation();
}

void printOrientation()
{
  //Serial.print(R.mean());
  //Serial.print("\t");
  //Serial.println(H.mean());
  double RG = gradientR*R.mean() + interceptR;
  double HG = gradientH*H.mean() + interceptH;
  Serial.print(RG);
  Serial.print("\t");
  Serial.print(HG);
  Serial.print("\t");
  Serial.println(180*atan2(HG,RG)/PI);
}

/*int printGforce(double XYZ[3]) {
  for (int i=0;i<3;i++)
  {
    if (XYZ[i] == XYZmid[i]) gforce[i] = 0;
    if (XYZ[i] < XYZmid[i]) gforce[i] = (XYZmid[i]-XYZ[i])/(XYZmid[i]-XYZmin[i]);
    else if (XYZ[i] > XYZmid[i]) gforce[i] = (XYZmid[i]-XYZ[i])/(XYZmax[i]-XYZmid[i]);
  }
  
  Serial.print(millis());
  Serial.print("\t");

  for (int j=0;j<3;j++)
  {
    if (gforce[j] > 0) Serial.print(" ");
    Serial.print(gforce[j]);
    Serial.print("\t");
  }
  Serial.println();
  //sprintf(buff,"%5.2f%10.2f%10.2f\n",XYZ[0],XYZ[1],XYZ[2]);
  //Serial.print(buff);
  /*Serial.print(gforce[0]);
  Serial.print("\t");
  Serial.print(gforce[1]);
  Serial.print("\t");
  Serial.println(gforce[2]);
  return 0;
}*/
