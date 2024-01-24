// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix.motorcontrol.ControlMode;
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
    CANSparkMax neito = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax neo = new CANSparkMax(6, MotorType.kBrushless);
    
    MotorController.add
   // ENCODERS: 
   /*
    * Un encoder es un sensor que nos ayuda a recolectar la información de nuestros motores.
    */

    // DECLARAR UN ENCODER:
    //RelativeEncoder neoteEncoder;

    // DECLARAR UN ENCODER externo:
    //RelativeEncoder boreEncoder;

  DifferentialDrive chassis = new DifferentialDrive(izq, der);

  Boolean isArcade = false; // tanque
  Boolean neitoPositive = true; // para mostrar en la shuffle board
  
  RelativeEncoder neitoEncoder; 
  double neitoSpeedPositive;
  double neitoSpeedNegative;
  double neitoSpeed;
    
  // NEO NEO
  CANSparkMax neote = new CANSparkMax(6, MotorType.kBrushless);
  double neoteSpeed;
  double neoteSpeedPositive;
  double neoteSpeedNegative;
  boolean neotePositive;

  
  Encoder boreEncoder = new Encoder(8, 7); 
  

  //la navX
  AHRS navX = new AHRS(SPI.Port.kMXP);
  double angle;
  
  // LIMIT SWITCHES:
  DigitalInput limitSwitch = new DigitalInput(6);
  
  // factor de converisión
  double diametroLlantaMetros = 0.1524; // --> metros
  
  double relacionBaja = 0.47;
  double relacionAlta = 1.36;
  double relacionBasica = 1.09;

  double relacionUsada = relacionBasica;

  double CountsPerRev = 2048;

  @Override
  public void robotInit() {
      // reset neo
      neote.restoreFactoryDefaults();
      neito.restoreFactoryDefaults();
      
  
    izq.setInverted(true);
    // jalar encoder alterno:

    // ASIGNAR BORE ENCODER:
    //boreEncoder = neote.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); // <-- depende de las CPR del encoder que andamos usando.
    /* CPRs:
     * MagEncoder = 1024
     * BoreEncoder = 8192
     * DATAZO ... el integrado del NEO = 42
     */
    //FACTOR DE CONVERSIÓN DE BORE ENCODER:
    //boreEncoder.setPositionConversionFactor(((Math.PI * diametroLlanta) / relacionUsada) / 10);

    //boreEncoder.getPosition(); 
    boreEncoder.getDistance();
    neitoEncoder = neito.getEncoder(); // variable para obtener los datos del encoder.
   
    //neoteEncoder = neote.getEncoder();
    
    //neoteEncoder.setPositionConversionFactor(0.5);


  }

  @Override
  public void robotPeriodic() {
    System.out.println(isArcade);
    System.out.println("bonsoir");
    SmartDashboard.putNumber("NavX Yaw", navX.getYaw());
    SmartDashboard.putNumber("NavX Angle", navX.getAngle());
    SmartDashboard.putNumber("NavX Pitch", navX.getPitch());
    SmartDashboard.putNumber("NavX Roll", navX.getRoll());
    SmartDashboard.putNumber("NavX AccelPitch", navX.getRawAccelX());

    SmartDashboard.putNumber("Neito Encoder Velocity", neitoEncoder.getVelocity()); // qué quiero específicamente del encoder.
    SmartDashboard.putNumber("Neito Encoder Position", neitoEncoder.getPosition());
    SmartDashboard.putBoolean("Neito Positive", neitoPositive);
    SmartDashboard.putNumber("Neito Speed Positive", neitoSpeedPositive);
    SmartDashboard.putNumber("Neito Speed Negative", neitoSpeedNegative);
    SmartDashboard.putNumber("Neote Speed", neoteSpeed);
    SmartDashboard.putBoolean("Neote Positive", neotePositive);

  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
      
  }

  @Override
  public void teleopInit() {
   //neoteEncoder.setPositionConversionFactor(2);
   boreEncoder.reset();


   // FACTOR DE CONVERSIÓN DE BORE ENCODER:
    boreEncoder.setDistancePerPulse((Math.PI * diametroLlantaMetros) / CountsPerRev);
    neitoEncoder.setPosition(0);
    
    //boreEncoder.setPosition(0);
    //neoteEncoder.setPosition(0);
    //boreEncoder.setPositionConversionFactor(((Math.PI * diametroLlanta) / relacionUsada) / 10);
    navX.reset();
  }

  @Override
  public void teleopPeriodic() {
    
    // NEOS se pueden seguir entre ellos.
    //neito.follow(neote);

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
    SmartDashboard.putNumber("Bore Encoder Distance:", boreEncoder.getDistance());
    SmartDashboard.putNumber("Bore Encoder Per Pulse:", boreEncoder.getDistancePerPulse());
    SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
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
