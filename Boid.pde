

class Boid
{
  
PVector position, acceleration, velocity, destination, crumb, direction;

ArrayList<PVector> breadcrumbs = new ArrayList<PVector>(512);

int start_time, end_time;
int opacity, randcount;

float max_speed, max_accel;
float max_ang_accel, max_rotation, steering_angular;
float orientation, roS, roD,r_rotS, r_rotD, angleChange, time_to_target,time_to_target_rot;
float distance, goal_speed, goal_rotation, elapsed_time;
float scale;

color crumbcolor = color(255, 255, 255);
color boidcolor = color(255, 255, 255);



  Boid(){
  
    position = new PVector(0, 0);
    velocity = new PVector(0,0);
    destination = new PVector(0,0);
    orientation = radians(0);
    acceleration = new PVector(0,0);
    direction = new PVector(0,0);
  
    max_speed = 10;
    max_accel = 10;
  
    roS = 3;
    roD = 500;
    r_rotS = radians(1);
    r_rotD = 5;
    max_ang_accel = 40;
    max_rotation  = 0.9;
    opacity = 255;
    time_to_target = 1.3;
    time_to_target_rot = 0.6;
    scale = 0.5;
    randcount = 0;
    
  }

  void run(){
  
  //moveRandom();
  //findMouse();
  seek();
  
  }

  void seek(){
    end_time = millis();
    check_border();
    
    elapsed_time = end_time - start_time;
    //breadcrumb_trail();
    elapsed_time = elapsed_time/100;
    
    update(arrive(destination));
    orient();
    moveBoid();
    start_time = millis();
    
  }


  void moveRandom(){
  
    if (randcount == 0){
    
      float left_fov = radians(-45);
      float right_fov = radians(45);
      float distance_of_rand_dest = random(80,120);
    
      destination.x = position.x + distance_of_rand_dest*cos(random(left_fov,right_fov) + orientation);
      destination.y = position.y + distance_of_rand_dest*sin(random(left_fov,right_fov) + orientation);
    
      randcount = 25;
    
    }
    
    randcount--; //pick new random point every randcount frames 
 
  }

  void findMouse(){
  
    if (mousePressed && (mouseButton == LEFT)){
    
      destination.x = mouseX;
      destination.y = mouseY;
      
    }
  
  }



  void check_border(){
  
    if ( position.x > width-12*scale){
      
      position.x = width-12*scale;
      velocity.x = 0;
      destination.x = width*0.9;
      //position.x = width;
      
    }

    if ( position.x < 12*scale){
      
      position.x = 12*scale;
      velocity.x = 0;
      destination.x = width*0.1;
      //position.x = 0;
      
    }
  
    if ( position.y > height-12*scale){
      
      position.y  = height-12*scale;
      velocity.y = 0;
      destination.y = height*0.9;
      //position.y = height;
      
    }

    if ( position.y < 12*scale){
      
      position.y = 12*scale;
      velocity.y = 0;
      destination.y = height*0.1;
      //position.y = 0;
      
    }

  }

  PVector arrive(PVector target){
  
    direction = PVector.sub(target,position);
    distance = direction.mag();

    check_roS();
    check_roD();

    PVector goal_velocity = direction;
    goal_velocity.normalize();
    goal_velocity.mult(goal_speed);
  
    PVector arrive_steering = PVector.sub(goal_velocity,velocity);
    arrive_steering.div(time_to_target);
  
    if (arrive_steering.mag() > max_accel){
      arrive_steering.normalize();
      arrive_steering.mult(max_accel);
  
    }
  
    return arrive_steering;
  
  
  }
  
  

  PVector flee(PVector target){
           
    PVector flee_steering = PVector.sub(position,target);
        
    if (flee_steering.mag() > max_accel){
    
      flee_steering.normalize();
      flee_steering.mult(max_accel);
    
     }
          
    return flee_steering;
             
  }
   
  void update(PVector steering){
  
    update_velocity(steering);
    update_position();
  
  }
  
  void update_velocity(PVector steering){
  
    if (steering.mag() > max_accel){
      steering.normalize();
      steering.mult(max_accel);
  
    }
  
    PVector a_t = PVector.mult(steering,elapsed_time);
    velocity.add(a_t);
 
  }

  void update_position(){
  
    PVector v_t = PVector.mult(velocity,elapsed_time);
    position.add(v_t);
 
  }
  



  void breadcrumb_trail(){

    int current_time = millis();
    
    if (current_time % 100 <= 20 ){
        
      breadcrumbs.add(PVector.mult(position,1));
      
    }

    int max_size = 511;

    for (int i = 0; i < breadcrumbs.size();i++) {
     
      crumb = breadcrumbs.get(i);

      if ((keyPressed == true) && (key == '.')){
        crumbcolor = color(255,255,255,0);
      } 
    
      fill(crumbcolor, (255*i/breadcrumbs.size())); 
    
      if ((keyPressed == true) && (key == '/')){
        crumbcolor = color(0,0,0);
      }
    
      noStroke();
      ellipse(crumb.x,crumb.y,4,4);  
 
    }


   if (breadcrumbs.size() >= max_size){
     
     breadcrumbs.remove(0);
     
    }
    
   if ((keyPressed == true) && (key == 'd')){
     
     breadcrumbs.clear();
    
    } 


  }


  void check_roS(){
 
    if (distance < roS){
      
       velocity.x = 0;
       velocity.y = 0;
       goal_speed = 0;

    }

  }
    
  
  void check_roD(){
  
    if (distance > roD){
    
      goal_speed = max_speed;
  
    }
  
    else {
    
      goal_speed = max_speed * distance/roD;
    
    }
  
  }
 


  void moveBoid(){
  
    drawBoid();

  }


  void orient(){
    if (direction.x == 0){return;}
    float target_orientation = atan2(direction.y,direction.x);
    float char_orientation = orientation;
    float rotation = target_orientation - char_orientation;
 
    float r = rotation % (2*PI);
  
    if (abs(r) <= PI){
    
    } 
    else if (r > PI){
      
      r -= (2*PI);
      
    }
    else {
    
      r += (2*PI);
  
    }
  
   float rotationSize = abs(r);
    
   if (rotationSize < r_rotS){
     
       r = 0;
       

    }
   
   if (rotationSize > r_rotD){
    
     goal_rotation = max_rotation;
  
    }
   else{
    
     goal_rotation = max_rotation * rotationSize/r_rotD;
    
    }
    
    goal_rotation *= (r/rotationSize);
  
    steering_angular = goal_rotation - r;
    steering_angular /= time_to_target_rot;
    float ang_accel = abs(steering_angular);
    
    if (ang_accel > max_ang_accel){
  
     steering_angular /= ang_accel; 
     steering_angular *= max_ang_accel;
   
    }
    
    orientation = orientation - (steering_angular* elapsed_time);
    
  }


 
  
  
  
  void drawBoid(){

    if ((keyPressed == true) && (key == 'F')){
      
      opacity = 0;
      
    }
    
    if ((keyPressed == true) && (key == 'f') ){
      
      opacity = 255;
     
    }
  
    pushMatrix();
    translate(position.x, position.y);
    rotate(orientation);
    noStroke();
    fill(boidcolor,opacity);
    pushMatrix();
    scale(scale);
    ellipse(0,0,24,24);
    triangle(5,-11,24,0,5,11);
    
    popMatrix();
    popMatrix();

  }


}
