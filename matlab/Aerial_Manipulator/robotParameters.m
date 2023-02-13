function  robotParameters % Funcion que tiene todos los puntos que forman el robot 
load 'Robot.mat' baseDrone soporteMotor helices baseBrazo primerEslabon segundoEslabon tercerEslabon cuartoEslabon;  % Carga los valores de cada elemento que compone el robot
global Robot;  % Variable global

Robot.baseDroneVertices=baseDrone.vertices'; % Variable global de vertices del cuerpo del robot
Robot.baseDroneFaces=baseDrone.faces; % Variable global de caras del cuerpo del robot

Robot.soporteMotorVertices=soporteMotor.vertices'; % Variable global de vertices del cuerpo del robot
Robot.soporteMotorFaces=soporteMotor.faces; % Variable global de caras del cuerpo del robot

Robot.helicesVertices=helices.vertices'; % Variable global de vertices del cuerpo del robot
Robot.helicesFaces=helices.faces; % Variable global de caras del cuerpo del robot

Robot.baseBrazoVertices=baseBrazo.vertices'; % Variable global de vertices del cuerpo del robot
Robot.baseBrazoFaces=baseBrazo.faces; % Variable global de caras del cuerpo del robot

Robot.primerEslabonVertices=primerEslabon.vertices'; % Variable global de vertices del cuerpo del robot
Robot.primerEslabonFaces=primerEslabon.faces; % Variable global de caras del cuerpo del robot

Robot.segundoEslabonVertices=segundoEslabon.vertices'; % Variable global de vertices del cuerpo del robot
Robot.segundoEslabonFaces=segundoEslabon.faces; % Variable global de caras del cuerpo del robot

Robot.tercerEslabonVertices=tercerEslabon.vertices'; % Variable global de vertices del cuerpo del robot
Robot.tercerEslabonFaces=tercerEslabon.faces; % Variable global de caras del cuerpo del robot

Robot.cuartoEslabonVertices=cuartoEslabon.vertices'; % Variable global de vertices del cuerpo del robot
Robot.cuartoEslabonFaces=cuartoEslabon.faces; % Variable global de caras del cuerpo del robot