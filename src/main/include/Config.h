
// possition du controller sur driver station
#define CONTROLLER_PORT_NO 0
//PWM pin led strip
#define LED_STRIP_PWM 9

//DRIVE---------------------------------------------------------

//Can id des moteur du drivetrain
#define LEFT_LEAD_MOTOR_ID 10
#define LEFT_FOLLOW_MOTOR_ID 18
#define RIGHT_LEAD_MOTOR_ID 1
#define RIGHT_FOLLOW_MOTOR_ID 6 

//BASCUL--------------------------------------------------------

//digital pin des deux encoder
#define ENCODER_BASCUL 1

//Can id motteur propulseur 
#define MOTOR_LANCER_AN_RIGHT 3 
#define MOTOR_LANCER_AN_LEFT 2

//valeur de la bascul la plus haute (60 degre et position de depart base)
#define MAX_VALUE_BASCUL -20
//valeur de la bascul le plus bas (pointe vers le bas)
#define MIN_VALUE_BASCUL -86

//Can id motor bascule
#define MOTOR_BASCUL_LEFT 19
#define MOTOR_BASCUL_RIGHT 20

//FEEDER---------------------------------------------------------

//digital pin des deux encoder
#define ENCODER_FEEDER 0

//Can id motteur position feeder
#define MOTOR_FEEDER 5

//Speed 0 a 1 pour motor_feeder
#define MOTOR_FEEDER_SPEED 0.2

//valeur de encoder du feeder
//prendre anneau
#define ENCODER_FEEDER_TAKE_VALUE 192 //189 valeur peut etre
//drop anneau dans lancer
#define ENCODER_FEEDER_LANCER_VALUE -6

//digital pin limit switch feeder
#define LIMIT_SWITCH 2

//Can id motteur roue feeder intake
#define MOTOR_FEEDER_INTAKE 4