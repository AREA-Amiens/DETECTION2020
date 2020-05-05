#include <Arduino.h>
#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 6

int detection(float x, float y, float angle, float distanceL, float angleL){

  float xL, yL, angleM1, angleM2, xM, yM, distanceM;

  Serial.print("angle :");
  Serial.println(angleL);
  if(angleL <= 46 || (angleL >= 134 && angleL <= 226) || angleL >= 314){

    // Prise en compte du sens du robot  //
    if (angle < 0) {
      angle += 360;
    }else{
      if (angle >= 360) {
        angle -= 360;
      }
    }

    angleL+=angle;
    if(angleL >= 360){
      angleL-= 360;
    }


    Serial.print("angleModif :");
    Serial.println(angleL);
    Serial.println("");

    // Conversion des valeurs obtenues sur les axes X et Y  //
    if (angleL >= 0 && angleL < 90){

      xL = distanceL*sin(angleL*PI/180);
      yL = distanceL*cos(angleL*PI/180);

    }else{
      if (angleL >= 90 && angleL < 180){

        xL = distanceL*sin(angleL*PI/180);
        yL = distanceL*(-cos(angleL*PI/180));

      }else{
        if (angleL >= 180 && angleL < 270){

          xL = distanceL*(-sin(angleL*PI/180));
          yL = distanceL*(-cos(angleL*PI/180));

        }else{
          if (angleL >= 270 && angleL < 360){

            xL = distanceL*(-sin(angleL*PI/180));
            yL = distanceL*cos(angleL*PI/180);

          }
        }
      }
    }
    // Conversion des valeurs obtenues sur les axes X et Y  //

    // TEST N - Détection du mat //
    xM = x;
    yM = y - 1500;

    distanceM = pow(xM,2) + pow(yM,2);
    distanceM = sqrt(distanceM);

    if(distanceM < 930){
      if(yM >= 0){ // Du coté de l'équipe jaune

        angleM1 = yM/xM;
        angleM1 = atan(angleM1)*180/PI;

        if(angleM1 > 26.565051){ // Du coté du mat

          angleM1 = xM/distanceM;
          angleM1 = acos(angleM1)*180/PI;

          xM = x + 222;
          yM = y - 1600;

          distanceM = pow(xM,2) + pow(yM,2);
          distanceM = sqrt(distanceM);
          angleM2 = xM/distanceM;
          angleM2 = acos(angleM2)*180/PI;

          // Déduction de la page d'angle selon la position du robot ( avec 3° de marge )

          angleM1 = 267 - angleM1 ;
          if(angleM1 < 0){
            angleM2+=360;
          }

          if(yM >= 0){
            angleM2 = 273 - angleM2;
            if(angleM2 < 0){
              angleM1+=360;
            }
          }else{
            angleM2 = 273 + angleM2;
            if(angleM2 >= 360){
              angleM1-=360;
            }
          }

          // Traitement
          if (angleM2 > angleM1) {
            if(angleL > angleM1 && angleL < angleM2){
              return 0;
            }
          }else{
            if(angleL > angleM1 && angleL < 360){
              return 0;
            }else{
              if (angleL >= 0 && angleL < angleM2) {
                return 0;
              }
            }
          }

        }else{ // Au centre
          xM = x + 222;
          yM = y - 1600;

          distanceM = pow(xM,2) + pow(yM,2);
          distanceM = sqrt(distanceM);
          angleM1 = xM/distanceM;
          angleM1 = acos(angleM1)*180/PI;

          yM = y - 1400;
          distanceM = pow(xM,2) + pow(yM,2);
          distanceM = sqrt(distanceM);
          angleM2 = xM/distanceM;
          angleM2 = acos(angleM2)*180/PI;

          // Déduction de la page d'angle selon la position du robot ( avec 3° de marge )
          yM = y - 1600;
          if (yM >= 0) {
            angleM1 = 273 - angleM1;
            if(angleM1 < 0){
              angleM1+=360;
            }
          }else{
            angleM1 = 273 + angleM1;
            if(angleM1 >= 360){
              angleM1-=360;
            }
          }

          angleM2 = 267 - angleM2 ;
          if(angleM2 < 0){
            angleM2+=360;
          }

          // Traitement
          if (angleM1 > angleM2) {
            if(angleL > angleM2 && angleL < angleM1){
              Serial.println("OUI");
              return 0;
            }
          }else{
            if(angleL > angleM2 && angleL < 360){
              return 0;
            }else{
              if (angleL >= 0 && angleL < angleM1) {
                return 0;
              }
            }
          }
        }

      }else{ // Du coté de l'équipe bleu

        yM*=-1;
        angleM1 = yM/xM;
        angleM1 = atan(angleM1)*180/PI;

        if(angleM1 > 26.565051){ // Du coté du mat

          angleM1 = xM/distanceM;
          angleM1 = acos(angleM1)*180/PI;

          xM = x + 222;
          yM = y - 1400;

          distanceM = pow(xM,2) + pow(yM,2);
          distanceM = sqrt(distanceM);
          angleM2 = xM/distanceM;
          angleM2 = acos(angleM2)*180/PI;

          // Déduction de la plage d'angle selon la position du robot ( avec 3° de marge )
          angleM1 = 273 + angleM1;
          if(angleM1 >= 360){
            angleM1-=360;
          }

          if(yM >= 0){
            angleM2 = 267 - angleM2;
            if(angleM2 < 0){
              angleM2+=360;
            }
          }else{
            angleM2 = 267 + angleM2;
            if(angleM2 >= 360){
              angleM2-=360;
            }
          }

          // Traitement
          if (angleM1 > angleM2) {
            if(angleL > angleM2 && angleL < angleM1){
              return 0;
            }
          }else{
            if(angleL > angleM2 && angleL < 360){
              return 0;
            }else{
              if (angleL >= 0 && angleL < angleM1) {
                return 0;
              }
            }
          }

        }else{ // Au centre

          xM = x + 222;
          yM = y - 1400;
          distanceM = pow(xM,2) + pow(yM,2);
          distanceM = sqrt(distanceM);
          angleM1 = xM/distanceM;
          angleM1 = acos(angleM1)*180/PI;

          yM = 1600 - y;
          distanceM = pow(xM,2) + pow(yM,2);
          distanceM = sqrt(distanceM);
          angleM2 = xM/distanceM;
          angleM2 = acos(angleM2)*180/PI;

          // Déduction de la page d'angle selon la position du robot ( avec 3° de marge )
          yM = y - 1400;
          if (yM >= 0) {
            angleM1 = 267 - angleM1;
            if(angleM1 < 0){
              angleM1+=360;
            }
          }else{
            angleM1 = 267 + angleM1;
            if(angleM1 >= 360){
              angleM1-=360;
            }
          }

          angleM2 = 273 + angleM2;
          if(angleM2 >= 360){
            angleM2-=360;
          }

          // Traitement
          if (angleM2 > angleM1) {
            if(angleL > angleM1 && angleL < angleM2){
              return 0;
            }
          }else{
            if(angleL > angleM1 && angleL < 360){
              return 0;
            }else{
              if (angleL >= 0 && angleL < angleM2) {
                return 0;
              }
            }
          }

        }

      }
    }
    // Détection du mat - TEST N //

    // Traitement des valeurs sur les axes X et Y selon les limites  //
    if(angleL >= 0 && angleL < 180){ // Droite
      if(xL > 2000-x){
        return 0;
      }else{
        if(angleL >= 0 && angleL < 90){ // Haut
          if(yL > 3000-y){
            return 0;
          }
        }else{ // Bas
          if(yL > y){
            return 0;
          }else{
            if(distanceL > 430 && distanceL < 930){ //PROCEDURE D'ESQUIVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              return 1;
            }else{
              return 0;
            }
          }
        }
      }
    }else{
      if(angleL >= 180 && angleL < 360){ // Gauche
        if(xL > x){
          return 0;
        }else{
          if(angleL >=180 && angleL < 270){ // Bas
            if(yL > y){
              return 0;
            }
          }else{ // Haut
            if(yL > 3000-y){
              return 0;
            }else{
              if(distanceL > 430 && distanceL < 930){ //PROCEDURE D'ESQUIVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                return 1;
              }else{
                return 0;
              }
            }
          }
        }
      }
    }
    // Traitement des valeurs sur les axes X et Y selon les limites  //

  }else{
    Serial.println("NON INTERESSANT");
    Serial.println("");
    return 0;

  }

}

void setup() {
  Serial.begin(9600);
  lidar.begin(Serial1);

  pinMode(RPLIDAR_MOTOR, OUTPUT);

  analogWrite(RPLIDAR_MOTOR, 250);
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13,HIGH);
  float x = 200, y = 1500, angle = 270, distanceL, angleL;
  int rep;

  if (IS_OK(lidar.waitPoint())) {

    distanceL = lidar.getCurrentPoint().distance;
    angleL = lidar.getCurrentPoint().angle;

    rep = detection(x,y,angle,distanceL,angleL);

  } else {
    //mettre en commentaire
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();

       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 250);
       delay(1000);
    }
  }
}
