
// possition du controller sur driver station
#define CONTROLLER_PORT_NO 0
//PWM pin led strip
#define LED_STRIP_PWM 2

//led strip led number
#define KLENGTH 150

//something for networkTable
#define TOPIC "/autonomous"

//DRIVE---------------------------------------------------------

//Can id des moteur du drivetrain
#define LEFT_LEAD_MOTOR_ID 10
#define LEFT_FOLLOW_MOTOR_ID 18
#define RIGHT_LEAD_MOTOR_ID 1
#define RIGHT_FOLLOW_MOTOR_ID 6 

//max speed during autonumus 0 to 1
#define DRIVE_MAX_SPEED_AUTO 1

//BASCUL--------------------------------------------------------

//digital pin des deux encoder
#define ENCODER_BASCUL 1

//valeur de la bascul la plus haute (60 degre et position de depart base)
#define MAX_VALUE_BASCUL -20
//valeur de la bascul le plus bas (pointe vers le bas)
#define MIN_VALUE_BASCUL -95

//Can id motor bascule
#define MOTOR_BASCUL_LEFT 19
#define MOTOR_BASCUL_RIGHT 20

#define BASCUL_VALUE_AMP -90

//PROPULSUER---------------------------------------------------------

//Can id motteur propulseur 
#define MOTOR_LANCER_AN_RIGHT 3 
#define MOTOR_LANCER_AN_LEFT 2

//Time before shooting in autonumus mode in seconds
#define WAIT_TIME_BEFORE_SHOOTING 2

//Time for shooting in autonumus mode in seconds
#define WAIT_TIME_SHOOTHING 2

//FEEDER---------------------------------------------------------

//digital pin des deux encoder
#define ENCODER_FEEDER 0

//Can id motteur position feeder
#define MOTOR_FEEDER 5

//valeur de encoder du feeder
//prendre anneau
#define ENCODER_FEEDER_TAKE_VALUE 180 //189 valeur peut etre
//drop anneau dans lancer
#define ENCODER_FEEDER_LANCER_VALUE -5 //val -5 depart

//angle when the feader is vertical
#define ENCODER_FEEDER_VERTICAL_ANGLE 90

//Speed 0 a 1 pour motor_feeder
#define FEEDER_DOWN_SPEED -0.2

//Speed 0 a 1 pour aspirer anneau
#define INTAKE_SPEED_SUCK 0.2

//speed of the intake to push the ring
#define INTAKE_PUSH_SPEED -0.5

//Speed 0 a 1 pour motor_feeder
#define FEEDER_UP_MAX_SPEED 0.4

#define FEEDER_DOWN_AMP_MAX_SPEED -0.2

//increment pour la vitesse de la remonter du feeder
#define FEEDER_SPEED_UP_INC 0.2

#define FEEDER_SPEED_AMP_INC 0.1

//value for the division for the speed of the feeder
#define ENCODER_FEEDER_SPEED_DIV ((ENCODER_FEEDER_TAKE_VALUE-ENCODER_FEEDER_VERTICAL_ANGLE)/5)

//digital pin limit switch feeder
#define LIMIT_SWITCH 2 

//Can id motteur roue feeder intake
#define MOTOR_FEEDER_INTAKE 4

#define AMP_POSITION_FEEDER_DOWN 233