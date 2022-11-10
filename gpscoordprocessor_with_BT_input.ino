#include <math.h>
#include "TinyGPS.h"

//  Pi/180
#define DEGTORAD 0.0174532925199432957f 
#define RADTODEG 57.295779513082320876f
#define TWOPI 6.28318530718f

struct Coord{
  float lat;
  float lon;
};

Coord carLoc;
Coord destLoc;
char newInstr = '0';

//haversine distance funtion
float makeDist(struct Coord &a, struct Coord &b){
  float dlat = (a.lat-b.lat) * DEGTORAD / 2;
  float dlon = (a.lon-b.lon) * DEGTORAD / 2;

  //sin^2(dlat) + ( sin^2(dlon) * cos(a.lat) * cos(a.lon) )
  float x = (sin(dlat)*sin(dlat)) + (sin(dlon)*sin(dlon) * cos(a.lat*DEGTORAD)*cos(b.lat*DEGTORAD));
  //arctan^2( sqrt(x) * sqrt(1-x) ) * 2
  float y = (atan2(sqrt(x), sqrt(1-x))) * 2;
  //convert dist to meters
  float z = 6372795 * y;

  return z;
}

//azimuth angle function (degrees from north)
float makeAngle(struct Coord &b, struct Coord &a){
  float alat = a.lat * DEGTORAD;
  float blat = b.lat * DEGTORAD;
  float dlon = (a.lon - b.lon) * DEGTORAD;
  float a1 = sin(dlon) * cos(alat);
  float a2 =  sin(blat) * cos(alat) * cos(dlon);
  a2 = (cos(blat) * sin(alat)) - a2;
  float ang = atan2(a1, a2);
  if(ang < 0.0){
    ang = ang + TWOPI;
  }
  ang = ang * RADTODEG;
  return ang;
}

Coord makePair(float &latP, float &lonP){
  Coord pairP;
  pairP.lat = latP;
  pairP.lon = lonP;
  return pairP;
}

Coord getDest(){
  float latd = 0.0;
  float lond = 0.0;

  Serial2.println("\nEnter Destination Latitude:");
  delay(7000);
   if (Serial2.available() > 0) {
     latd = Serial2.parseFloat();
  }
  Serial2.println(latd);
  Serial2.println("\nEnter Destination Longitude:");
  delay(7000);
   if (Serial2.available() > 0) {
     lond = Serial2.parseFloat();
  }
  Serial2.println(lond);

  Coord pairD = makePair(latd, lond);
  return pairD;
}

char doCalcs(){
  Serial2.print("\nEnter GPS coordinates in the form xx.xxxxxx \n");

  float lat1 = 40.24056;
  float lon1 = -83.03100;

  float distance = 0.0;
  float angle = 0.0;

  carLoc = makePair(lat1, lon1);
  destLoc = getDest();

  distance = makeDist(carLoc, destLoc);
  angle = makeAngle(carLoc, destLoc);

  Serial2.println("\nThe car must move "+ (String)distance +" meters after adjusting its angle "+ (String)angle +" degrees from north.\n");
  return '0';
}

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(9600);
  delay(3000);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial2.println("If you wish to update to new coordinates- press (y):");
  if (Serial2.available() > 0) {
    delay(5000);
    newInstr = Serial2.read();
  }
  if(newInstr == 'y'){
    newInstr = doCalcs();
  }
    delay(10000);
  

}
