import processing.serial.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.InputEvent;

Serial myPort;

int mouse_press_flag = 0;
boolean temp;

void setup(){
  println(Serial.list());
  println(Serial.list()[7]);
  String portName=Serial.list()[7];
  myPort=new Serial(this,portName,9600);

  myPort.bufferUntil('\n');
}

void draw(){
  serialEvent(myPort);
}

void serialEvent(Serial myPort){
  String inputString=myPort.readStringUntil('\n');
  inputString=trim(inputString);
  RobotTest(inputString);
}

void RobotTest(String temp){
  try{
    Robot robot=new Robot();
    PointerInfo M_pointer = MouseInfo.getPointerInfo();
  //  println(M_pointer.getLocation());
   // println(temp);
    //robot.setAutoDelay(3);
    int x=M_pointer.getLocation().x;
    int y=M_pointer.getLocation().y;
    
    if (temp.equals("S"))
      change_pen(robot,1323,578,x,y);
 
    else if (temp.equals("M"))
    change_pen(robot,1367,579,x,y);
    
    else if (temp.equals("L"))
      change_pen(robot,1412,576,x,y);
      
    else if (temp.equals("BLACK"))
      change_pen(robot,1316,209,x,y);
     
    else if (temp.equals("BLUE"))
      change_pen(robot,1516,311,x,y);
      
    else if (temp.equals("RED"))
      change_pen(robot,1367,262,x,y);
      
    else if (temp.equals("WRITE"))
      change_pen_mode(robot,1515,112,x,y);
      
    else if (temp.equals("ERASE"))
      change_pen_mode(robot,1668,109,x,y);
   
    if(temp.equals("1")){
      robot.mousePress(InputEvent.BUTTON1_DOWN_MASK);
      }
      else if(temp.equals("0")){
     robot.mouseRelease(InputEvent.BUTTON1_DOWN_MASK); 
    }
    if(temp.equals("UP")){
        robot.mouseMove(x,y-1);
    }
    if(temp.equals("DOWN")){
        robot.mouseMove(x,y+1);
    }
    if(temp.equals("RIGHT")){
        robot.mouseMove(x+1,y);
    }
    if(temp.equals("LEFT")){
        robot.mouseMove(x-1,y);
    }
    
    
    
    
      
   // else if(temp.equals("1"))
       
      
    
  }catch(Exception e){}
}
void change_pen(Robot robot,int x_val,int y_val,int x,int y){
   for(int i=0;i<10;i++){
      robot.mouseMove(1584,114);//it is hard to find its location in one-time
      }
      robot.mousePress(InputEvent.BUTTON1_DOWN_MASK);
      robot.mouseRelease(InputEvent.BUTTON1_DOWN_MASK);
      
      for(int i=0;i<15;i++){
      robot.mouseMove(x_val,y_val);//it is hard to find its location in one-time
      }
      robot.delay(100);
      robot.mousePress(InputEvent.BUTTON1_DOWN_MASK); 
      robot.mouseRelease(InputEvent.BUTTON1_DOWN_MASK);//click the left button
      
   for(int i=0;i<15;i++){
      robot.mouseMove(1584,114);//it is hard to find its location in one-time
      }
      robot.delay(100);
      robot.mousePress(InputEvent.BUTTON1_DOWN_MASK); 
      robot.mouseRelease(InputEvent.BUTTON1_DOWN_MASK);//click the left button
      
      for(int i=0;i<15;i++){
        robot.mouseMove(x,y);//go back to the location before click button1
      }   
}
void change_pen_mode(Robot robot,int x_val,int y_val,int x,int y){
   for(int i=0;i<10;i++){
      robot.mouseMove(x_val,y_val);//it is hard to find its location in one-time
      }
      
      robot.mousePress(InputEvent.BUTTON1_DOWN_MASK);
      robot.mouseRelease(InputEvent.BUTTON1_DOWN_MASK);

      for(int i=0;i<10;i++){
        robot.mouseMove(x,y);//go back to the location before click button1
      }  
}
