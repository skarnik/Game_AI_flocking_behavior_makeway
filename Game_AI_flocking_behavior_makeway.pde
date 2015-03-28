
//Sushrut Karnik
//CSC484
//flocking
//Change number of flock boids and leaders if needed, but parameters 
//will have to be tweaked
//Click on any point to make leaders seek that position
Flock f1;

void setup(){
  
  size(1024,600); 
  smooth();

        // Arguments: (No. of flock boids, no. of leaders)
  f1 = new Flock(150,2);



}

void draw() {  
  
  background(0,150,250);

  f1.run_flock();








}



