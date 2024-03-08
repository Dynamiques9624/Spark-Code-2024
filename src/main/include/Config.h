
// possition du m_controller sur driver station
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

//value for the sensibility of turning
//value increment and limitation
#define IX 0.6
//multiplicateur 
#define FX 1

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

//position of the bascul when whant to scorre in the amp
#define BASCUL_VALUE_AMP -90 

//position of the bascul when want the feeder to go in amp position
#define BASCUL_VALUE_DEPLOY_FEEDER_AMP -35

//PROPULSUER---------------------------------------------------------

//Can id motteur propulseur 
#define MOTOR_LANCER_AN_RIGHT 3 
#define MOTOR_LANCER_AN_LEFT 2

//Time before shooting in autonumus mode in seconds
#define WAIT_TIME_BEFORE_SHOOTING 3

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
#define ENCODER_FEEDER_LANCER_VALUE -3 //-4 val depart avant changement -5

//angle when the feader is vertical
#define ENCODER_FEEDER_VERTICAL_ANGLE 90

//Speed 0 a 1 pour aspirer anneau
#define INTAKE_SPEED_SUCK 0.4

//speed of the intake to push the ring
#define INTAKE_PUSH_SPEED -0.5

//Speed 0 a 1 pour motor_feeder
#define FEEDER_DOWN_SPEED -0.54 ///////-0.4

//Speed 0 a 1 pour motor_feeder
#define FEEDER_UP_MAX_SPEED 0.54 ///////0.4

#define FEEDER_UP_SPEED_AMP 0.50

//increment pour la vitesse de la remonter du feeder
#define FEEDER_SPEED_UP_INC 0.27 ////// 0.2

//speed of feeder when going in amp position
#define FEEDER_DOWN_AMP_MAX_SPEED -0.27 ///// -0.27

#define FEEDER_SPEED_AMP_INC 0.135 //////0.1

//value for the division for the speed of the feeder
#define ENCODER_FEEDER_SPEED_DIV ((ENCODER_FEEDER_TAKE_VALUE-ENCODER_FEEDER_VERTICAL_ANGLE)/5)

//digital pin limit switch feeder
#define LIMIT_SWITCH 2 

//Can id motteur roue feeder intake
#define MOTOR_FEEDER_INTAKE 4

//Position of the feeder when drop ring in amp
#define AMP_POSITION_FEEDER_DOWN 233

//value for when the bascul go back to normal position after amp
#define FEEDER_VALUE_DEPLOY_BASCUL_AMP 187

//value for shake of the ring
#define ENCODER_FEEDER_STOP_SHAKE_VALUE (ENCODER_FEEDER_LANCER_VALUE + 70)
#define SHAKE_RING_OUT_TIME 30
#define SHAKE_RING_INTAKE 0.1


//configue pour pathplanner
#define WHEEL_DISTANCE 0.559_m
#define MOTOR_MAX_RPM 3000 // 5310
#define MOTOR_GR 10.71
#define WHEEL_DIAM_IN 6.0
#define WHEEL_CIRC_IN 18.85

#define DEBUG

//#define PATHPLANNER