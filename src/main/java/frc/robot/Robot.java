// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* IMPORTANTE:
 * - Todo lo que está entre estos símbolos está comentado, por lo que no será interpretado
    como código por el compilador.
    - Para hacer comentarios en tu código usa "//" Se recomienda comentar lo que vamos haciendo 
    paso a paso para no olvidar cuando hagas otro código.
    - Antes de iniciar a programar, asegúrate de que tienes instaladas las librerías necesarias, para eso:
        1. Ve a la "W" en la esquina superior derecha de WPI Lib y haz click.
        2. En el menú desplegable, busca "Manage Vendor Libraries".
        3. Da click en "Install new Libraries (online)". Deberían aparecerte las librerías que ya instalaste 
        en tu compu con ayuda de los mentores :) Si no, acércarte a ellos para que tengas todo listo.
        4. Para asegurarte de que ya está todo listo, ve al árbol de archivos (parte izquierda), da click en la flecha de 
        "vendordeps" y podrás ver una lista de archivos "".json".
        5. Ahora sí, a codear :)
*/

// ESTOS COMENTARIOS LOS GENERA WPI LIB AUTOMÁTICAMENTE PARA EXPLICAR CIERTAS PARTES DEL CÓDIGO:
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  /**
   * "This function is run when the robot is first started up and should be used for any
   * initialization code."
  */
  // INITIALIZATION: crear los componentes que usaremos en el robot, como motores, nuestro control y sensores.

  // Inicializar un motor:
  // NombreDelControlador nombre_del_motor = new NombreDelControlador(argumentos);

  //Ejemplo: inicializar un TalonSRX -> para motor CIM
  // WPI_TalonSRX motor1 = new WPI_TalonSRX(1);

  //Ejemplo: inicializar un CanSparkMax -> para motor NEO o NEO550.
  //CANSparkMax motor1 = new CANSparkMax(deviceId: , MotorType.kBrushless); --> hay dos tipos de motores, si quieres saber más,pregúntale a alguien de eléctrica.

  //Inicializar un control: (hay más maneras, pero está es muy sencilla)
  //Joystick control = new Joystick(0); 

  /* ojo: todos los nombres puestos aquí son de ejemplo. Tú puedes nombrar tus componentes como quieras, pero la buena
   * práctica es darles nombres significativos. Ejemplo: NO "salchichamotor1", PERO SÍ "motorAvanceDerecho" :D
   */
   

  /* OJO: para saber qué argumentos requiere cada controlador, pon
     tu cursor encima del nombre del controlador. 
     En la parte de "Parameters" verás lo que te piden.
  */ 
  
  // AHORA ES TU TURNO: 
  /*
   * Crea:
   * 1. 2 motores NEO: uno de avance y uno de giro.
   * 2. 2 motores CIM: uno de avance y uno de giro.
   * 3. 2 Controles: uno para driver de chasis y otro para driver de mecanismos.
   */

   /*++++++++++++++++++++++++++++++++++ tu código aquí ++++++++++++++++++++++++++++++++++++++++++++++++ */
 







   /*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

   // CREAR UN CHASIS:
   /*
    * Como puedes ver en cualquier robot de FRC, un chasis está compuesto por más de dos motores que hacen lo mismo (están agrupados), 
      por lo que debemos hacer que nuestros motores actúen a la par.
    */

    // Follower:
    // motor.follow(motorASeguir);

    
    
   // ENCODERS: 
   /*
    * Un encoder es un sensor que nos ayuda a recolectar la información de nuestros motores.
    */

    // DECLARAR UN ENCODER:
    // RelativeEncoder nombreDelEncoder; ---> sólo indicamos que hacemos un nuevo objeto, más adelante le asignaremos un valor.

    // DECLARAR UN ENCODER externo (DIRECTO AL SPARK):
    // RelativeEncoder boreEncoder; --> se recomienda dejarle el nombre de "boreEncoder"

    // DECLARAR UN ENCODER EXTERNO:
    // Encoder nombre = new Encoder(puerto1, puerto2);

    /* 

    /* DECLARAR LA NAVX: la navX es una IMU (Inner Mass Unit) que nos devuelve los valores de Yaw, Pitch y Roll del robot. Es decir:
    Como se mueve en x, y, y tridimensionalmente (de arriba hacia abajo)
    AHRS navX = new AHRS(SPI.Port.kMXP); --> el argumento es el puerto SPI. Este puerto es el grandote encima de la RoboRIO.
    
    // Crear variables para obtener datos de la navX:
    double angle; --> double es un tipo de dato que adopta decimales o enteros con decimal.
    double yaw;
    double pitch;

    */
  
  // LIMIT SWITCHES:
  //DigitalInput limitSwitch = new DigitalInput(6);
  
  // DECLARAR FACTORES DE CONVERSIÓN PARA UN BORE ENCODER:
   
  double diametroLlantaMetros = 0.1524; // --> metros
  
  double relacionBaja = 0.47;
  double relacionAlta = 1.36;
  double relacionBasica = 1.09;

  double relacionUsada = relacionBasica;

  double CountsPerRev = 2048;

  
 

  
  
 
  @Override
  public void robotInit() {
    /* 
      // Resetear un motor a sus valores originales:
      motor.restoreFactoryDefaults();
    
      
      //Invertir un motor:
      motor.setInverted(true);

    // jalar encoder alterno:

    // ASIGNAR UN BORE ENCODER A UN MOTOR:
    //boreEncoder = motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); // <-- el último número depende de las CPR del encoder que andamos usando.
    /* CPRs:
     * MagEncoder = 1024
     * BoreEncoder = 8192
     * DATAZO ... el integrado del NEO = 42
     */

    //FACTOR DE CONVERSIÓN DE BORE ENCODER: para obtener valores en metros.
    //boreEncoder.setPositionConversionFactor(((Math.PI * diametroLlanta) / relacionUsada) / 10);

    //OBTENER DATOS DEL ENCODER:
    /* 
      boreEncoder.getPosition(); 
      boreEncoder.getDistance();

    
    */
    
    // OBTENER DATOS DEL ENCODER INTEGRADO DE LOS MOTORES:
    /* 
      neoteEncoder = neote.getEncoder();

    */

    // te toca:
    /*
     * 1. Invierte un motor y resetealo a sus factory defaults.
     * 2. Declara un BoreEncoder.
     * 3. Asigna un factor de conversión dependiendo de su tipo y CPR.
     * 
     */
    

  }

  @Override
  public void robotPeriodic() {
    /* 
    OJO: recuerda que todo lo que sea "Periodic" se repetirá continuamente dependiendo del periodo del juego en el que nos encontremos.
    RobotPeriodic() está ejecutándose siempre.

    // IMPRIMIR DATOS DE NUESTROS COMPONENTES.

    // imprimir en la consola:
    System.out.println(variable);
    
    // imprimir en el SmartDashboard:
    SmartDashboar.put(...)("nombre", variable); --> explora todoo lo que pueedes obtener de tus componentes escribiendo un "." y haciendo hover sobre la función.

    Ejemplo:
    SmartDashboard.putNumber("NavX Angle", navX.getAngle());
    SmartDashboard.putBoolean("Motor Invertido", motor.isInverted());
    
    */

    // TE TOCA:
    /*
     * 1. Imprime el roll, pitch y yaw de la NavX en el SmartDashboard.
     * 2. Imprime en la consola si tus motores están invertidos o no.
     * 3. RETO: escribe en la función @Override adecuada una línea de código que diga tu nombre SOLAMENTE en el Teleoperado (todo el teleoperado).
     */
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
      
  }

  @Override
  public void teleopInit() {
     /*
   // CADA VEZ QUE INICIAMOS EL TELEOPERADO es necesario resetear la posición de nuestros encoders:
      boreEncoder.reset();
      motor.setPosition(0);

    // también la navX: ¿Cómo lo harías tú?
    // Busca una fucnión que lo haga.
  
    */

   // FACTOR DE CONVERSIÓN DE BORE ENCODER:
     boreEncoder.setDistancePerPulse((Math.PI * diametroLlantaMetros) / CountsPerRev);
    
    
  


    
  }

  @Override
  public void teleopPeriodic() {

    // Aquí ejecutaremos todas las acciones que queremos hacer en el teleoperado, como: movernos, anotar, detectar targets, etc...
    
    // NEOS se pueden seguir entre ellos.
    //  neito.follow(neote);
    /* 
    if (control_chassis.getRawButton(1)) {
      isArcade = true; // arcade
    } else if (control_chassis.getRawButton(3)){
      isArcade = false; // tanque
    }

    

    if (isArcade) { // no está en arcade
      chassis.arcadeDrive(control_chassis.getRawAxis(1) * 0.5, control_chassis.getRawAxis(4) * 0.5);
    } else {
      chassis.tankDrive(control_chassis.getRawAxis(1) * 0.5, control_chassis.getRawAxis(5) * 0.5);
    }

    //double angle = navX.getAngle();
    /* 
    if (angle > 90 && angle < 110){
      neito.set(0.1);
    } else if (angle > 180 && angle < 210){
      neito.set(-0.1);
    } else {
      neito.set(0);
    }
    */
    //neito.set(ControlMode.PercentOutput, speed); // --> creo que así le puedes poner %



    /* 
    if (control_chassis.getRawAxis(2) > 0 && control_chassis.getRawAxis(3) == 0 ){
      neitoSpeedPositive = control_chassis.getRawAxis(2) * 0.1;
      neitoPositive = true;
      neitoSpeed = neitoSpeedPositive;
      
    
    } else if (control_chassis.getRawAxis(3) > 0 && control_chassis.getRawAxis(2) == 0) {
      neitoSpeedNegative = control_chassis.getRawAxis(3) * -0.1;

      neitoSpeed = neitoSpeedNegative;
      neitoPositive = false;
    } else {
      neitoSpeed = 0;
    }
    
    
    */

    // neito
    /* 
    neitoSpeed = control_chassis.getRawAxis(0) * 0.1;
    neito.set(neitoSpeed);
    SmartDashboard.putNumber("SPEED", neitoSpeed);
    */

    // neote
    /* 
    if (control_chassis.getRawAxis(2) > 0 && control_chassis.getRawAxis(3) == 0){
      neoteSpeed = control_chassis.getRawAxis(2) * 0.3;
      neotePositive = true;
    } else if (control_chassis.getRawAxis(3) > 0 && control_chassis.getRawAxis(2) == 0){
      neoteSpeed = control_chassis.getRawAxis(3) * -0.3;
      neotePositive = false;

    } else {  
  
      neoteSpeed = 0;
    }
    */
    /* 
    // LIMIT SWITCH:
    if (!limitSwitch.get()) { // si es false -> APRETADO -> neito para.
      neoteSpeed = 0;
    } else { // si es true -> LIBRE -> neito se mueve.
      neoteSpeed = control_chassis.getRawAxis(0) * 0.1;
      
    }
    */


    //neoteSpeed = control_chassis.getRawAxis(0) * 0.075;
    //neote.set(neoteSpeed);      
    //SmartDashboard.putNumber("Neotnencoder", neoteEncoder.getPosition());
    //SmartDashboard.putNumber("SPEeD", neoteEncoder.getVelocity());

    //SmartDashboard.putNumber("Neote Bore Encoder", boreEncoder.getPosition());
    /* 
    SmartDashboard.putNumber("Bore Encoder Distance:", boreEncoder.getDistance());
    SmartDashboard.putNumber("Bore Encoder Per Pulse:", boreEncoder.getDistancePerPulse());
    SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    */
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
