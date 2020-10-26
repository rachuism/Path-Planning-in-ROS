#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

#define PI 3.14159265


#define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))

using namespace std;

double outputs[2048];
double x_transformada;
double y_transformada;
//int aux_count = 0;

//nav_msgs::Path path_map;
geometry_msgs::PoseStamped goal_point;
geometry_msgs::PoseStamped goal_point1;
geometry_msgs::PoseStamped goal_point2;

nav_msgs::OccupancyGrid map_;
nav_msgs::Odometry current_odom;
nav_msgs::Odometry current_odom1;
nav_msgs::Odometry current_odom2;
nav_msgs::Path trajectory;
nav_msgs::Path trajectory1;
nav_msgs::Path trajectory2;
geometry_msgs::PoseArray trajectory_array;
geometry_msgs::PoseArray trajectory_array1;
geometry_msgs::PoseArray trajectory_array2;
visualization_msgs::Marker trajectory_point;
visualization_msgs::Marker trajectory_point1;
visualization_msgs::Marker trajectory_point2;

geometry_msgs::Pose auxiliar_point[3];



bool goal_init = false;
bool map_init = false;

double factorialLookup[33] = {
    1.0,
    1.0,
    2.0,
    6.0,
    24.0,
    120.0,
    720.0,
    5040.0,
    40320.0,
    362880.0,
    3628800.0,
    39916800.0,
    479001600.0,
    6227020800.0,
    87178291200.0,
    1307674368000.0,
    20922789888000.0,
    355687428096000.0,
    6402373705728000.0,
    121645100408832000.0,
    2432902008176640000.0,
    51090942171709440000.0,
    1124000727777607680000.0,
    25852016738884976640000.0,
    620448401733239439360000.0,
    15511210043330985984000000.0,
    403291461126605635584000000.0,
    10888869450418352160768000000.0,
    304888344611713860501504000000.0,
    8841761993739701954543616000000.0,
    265252859812191058636308480000000.0,
    8222838654177922817725562880000000.0,
    263130836933693530167218012160000000.0
};

// just check if n is appropriate, then return the result
double factorial( int n ) {

    if ( n < 0 ) { printf( "ERROR: n is less than 0\n" ); }
    if ( n > 32 ) { printf( "ERROR: n is greater than 32\n" ); }

    return factorialLookup[n]; /* returns the value n! as a SUMORealing point number */
}

double Ni( int n, int i ) {
    double ni;
    double a1 = factorial( n );
    double a2 = factorial( i );
    double a3 = factorial( n - i );
    ni =  a1 / ( a2 * a3 );
    return ni;
}

// Calculate Bernstein basis
double Bernstein( int n, int i, double t ) {
    double basis;
    double ti; /* t^i */
    double tni; /* (1 - t)^i */

    /* Prevent problems with pow */

    if ( t == 0.0 && i == 0 ) {
        ti = 1.0;
    } else {
        ti = pow( t, i );
    }
    if ( n == i && t == 1.0 ) {
        tni = 1.0;
    } else {
        tni = pow( ( 1 - t ), ( n - i ) );
    }
    //Bernstein basis
    basis = Ni( n, i ) * ti * tni;
    return basis;
}

void Bezier2D( double b[], int bCount, int cpts, double p[] ) {
    int npts = bCount / 2;
    int icount, jcount;
    double step, t;

    // Calculate points on curve

    icount = 0;
    t = 0;
    step = (double)1.0 / ( cpts - 1 );

    for ( int i1 = 0; i1 != cpts; i1++ ) {
        if ((1.0 - t) < 5e-6) {
            t = 1.0;
        }
        jcount = 0;
        p[icount] = 0.0;
        p[icount + 1] = 0.0;
        for ( int i = 0; i != npts; i++ ) {
            double basis = Bernstein(npts - 1, i, t);
            p[icount] += basis * b[jcount];
            p[icount + 1] += basis * b[jcount + 1];
            jcount = jcount +2;
        }

        icount += 2;
        t += step;
    }
}


void goalPointCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg )
{
  goal_point = *goal_msg;  // goal_msgs.pose.position.x / .y/ .z /// tf::quaternion2theta
  goal_init = true;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg )
{
  map_ = *map_msg;
  map_init = true;
}

void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  current_odom = *odom_msg;
}

nav_msgs::Path generate_path(geometry_msgs::PoseStamped goal_point, nav_msgs::Odometry current_odom, geometry_msgs::Pose auxiliar_point[3], tf::StampedTransform transform)
{
  nav_msgs::Path path_map;
  geometry_msgs::PoseArray array_map;

  double outputs[2048];
  // advanced curve


  double inputs[8] = {
      (double)current_odom.pose.pose.position.x, (double)(current_odom.pose.pose.position.y),
      (double)current_odom.pose.pose.position.x+5.0, (double)(current_odom.pose.pose.position.y+5.0), //Para empezar con la orientacion adecuada
      //(double)auxiliar_point[0].position.x, (double)auxiliar_point[0].position.y,
      //(double)auxiliar_point[1].position.x, (double)auxiliar_point[1].position.y,
      //(double)auxiliar_point[2].position.x, (double)auxiliar_point[2].position.y,
      (double)goal_point.pose.position.x-5.0, (double)(goal_point.pose.position.y+5.0),
      (double)goal_point.pose.position.x, (double)goal_point.pose.position.y
  };


  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped pose_map;
  geometry_msgs::Pose pose_array;


     Bezier2D( inputs, ARRAY_SIZE( inputs ), 1024, outputs );

  // generate trayectory

     path_map.header.frame_id = "map";
     path_map.header.stamp = ros::Time::now();


     for ( int x = 0; x < 2048; x += 2 ) {


         //Por cuaternios

         //double w = cos (0.5*angle*PI/180);
         //double x = p.x * sin (0.5*angle*PI/180);
         //double y = p.y * sin (0.5*angle*PI/180);
         //double z = p.z * sin (0.5*angle*PI/180);

         pose.pose.position.x = outputs[x];
         pose.pose.position.y = outputs[x+1];
         pose.pose.position.z = 0;

         pose.pose.orientation.x = 0;
         pose.pose.orientation.y = 0;
         pose.pose.orientation.z = 0;
         pose.pose.orientation.w = 1.0; //Significa que no hay rotación.

         //Otro sistema de poses en referencia a /map

         pose_map.pose.position.x = pose.pose.position.x + transform.getOrigin().x();
         pose_map.pose.position.y = pose.pose.position.y + transform.getOrigin().y();
         pose_map.pose.position.z = pose.pose.position.z + transform.getOrigin().z();

         pose_map.pose.orientation.x = 0;
         pose_map.pose.orientation.y = 0;
         pose_map.pose.orientation.z = 0;
         pose_map.pose.orientation.w = 1.0;

         pose_array.position.x = pose.pose.position.x + transform.getOrigin().x();
         pose_array.position.y = pose.pose.position.y + transform.getOrigin().y();
         pose_array.position.z = pose.pose.position.z + transform.getOrigin().z();


         pose_array.orientation.x = 0;
         pose_array.orientation.y = 0;
         pose_array.orientation.z = 0;
         pose_array.orientation.w = 1.0;

         //array_map.poses.push_back(pose_array);
         path_map.poses.push_back(pose_map);

         //path.poses[].pose.position

         //chatter_pub.publish(msgy);

         //Publicar un array de poses
     }


  return path_map;
}


geometry_msgs::PoseArray generate_array(geometry_msgs::PoseStamped goal_point, nav_msgs::Odometry current_odom, geometry_msgs::Pose auxiliar_point[3], tf::StampedTransform transform)
{
  nav_msgs::Path path_map;
  geometry_msgs::PoseArray array_map;
  double hip;
  double theta;
  double toggle = 1;

  double outputs[2048];
  // advanced curve


  double inputs[8] = {
      (double)current_odom.pose.pose.position.x, (double)(current_odom.pose.pose.position.y),
      (double)current_odom.pose.pose.position.x+5.0, (double)(current_odom.pose.pose.position.y+5.0), //Para empezar con la orientacion adecuada
      //(double)auxiliar_point[0].position.x, (double)auxiliar_point[0].position.y,
      //(double)auxiliar_point[1].position.x, (double)auxiliar_point[1].position.y,
      //(double)auxiliar_point[2].position.x, (double)auxiliar_point[2].position.y,
      (double)goal_point.pose.position.x-5.0, (double)(goal_point.pose.position.y+5.0),
      (double)goal_point.pose.position.x, (double)goal_point.pose.position.y};


  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped pose_map;
  geometry_msgs::Pose pose_array;


     Bezier2D( inputs, ARRAY_SIZE( inputs ), 1024, outputs );

  // generate trayectory

     array_map.header.frame_id = "map";
     array_map.header.stamp = ros::Time::now();


     for ( int x = 0; x < 1900; x += 2 ) { //for ( int x = 0; x < 2048; x += 2 ) {


         //Por cuaternios

         //double w = cos (0.5*angle*PI/180);
         //double x = p.x * sin (0.5*angle*PI/180);
         //double y = p.y * sin (0.5*angle*PI/180);
         //double z = p.z * sin (0.5*angle*PI/180);

         pose.pose.position.x = outputs[x];
         pose.pose.position.y = outputs[x+1];
         pose.pose.position.z = 0;

         pose.pose.orientation.x = 0;
         pose.pose.orientation.y = 0;
         pose.pose.orientation.z = 0;
         pose.pose.orientation.w = 1.0; //Significa que no hay rotación.

         //Otro sistema de poses en referencia a /map

         pose_map.pose.position.x = pose.pose.position.x + transform.getOrigin().x();
         pose_map.pose.position.y = pose.pose.position.y + transform.getOrigin().y();
         pose_map.pose.position.z = pose.pose.position.z + transform.getOrigin().z();

         pose_map.pose.orientation.x = 0;
         pose_map.pose.orientation.y = 0;
         pose_map.pose.orientation.z = 0;
         pose_map.pose.orientation.w = 1.0;

         pose_array.position.x = pose.pose.position.x + transform.getOrigin().x();
         pose_array.position.y = pose.pose.position.y + transform.getOrigin().y();
         pose_array.position.z = pose.pose.position.z + transform.getOrigin().z();


         //We only rotate around the z coordinate, the only parameters changed are z and w
         if((outputs[x+1] - outputs[x+17])>0){
             toggle = toggle*-1;
         }

         hip = sqrt(pow(outputs[x] - outputs[x+16],2) + pow(outputs[x+1]-outputs[x+17],2));
         theta = acos((outputs[x+16]-outputs[x+0])/hip);

        /* cout << "Theta es" << theta << endl;
         cout << "La diferencia es" << outputs[x+17]-outputs[x+1] << endl;
         cout << "la hipotenusa es" << hip << endl;*/


         pose_array.orientation.x = 0;
         pose_array.orientation.y = 0;
         pose_array.orientation.z = toggle*sin(0.5*theta);
         pose_array.orientation.w = cos(0.5*theta);

         array_map.poses.push_back(pose_array);
         toggle = 1;
         //path_map.poses.push_back(pose_map);

         //path.poses[].pose.position

         //chatter_pub.publish(msgy);

         //Publicar un array de poses
     }




  return array_map;
}

visualization_msgs::Marker generate_point(geometry_msgs::PoseStamped goal_point, nav_msgs::Odometry current_odom, geometry_msgs::Pose auxiliar_point[3], tf::StampedTransform transform){
    //double outputs[2048];
    double outputs[600];
    // advanced curve


    double inputs[8] = {
        (double)current_odom.pose.pose.position.x, (double)(current_odom.pose.pose.position.y),
        (double)current_odom.pose.pose.position.x+3.0, (double)(current_odom.pose.pose.position.y), //Para empezar con la orientacion adecuada
        //(double)auxiliar_point[0].position.x, (double)auxiliar_point[0].position.y,
        //(double)auxiliar_point[1].position.x, (double)auxiliar_point[1].position.y,
        //(double)auxiliar_point[2].position.x, (double)auxiliar_point[2].position.y,
        (double)goal_point.pose.position.x-5.0, (double)(goal_point.pose.position.y),
        (double)goal_point.pose.position.x, (double)goal_point.pose.position.y
    };

    Bezier2D( inputs, ARRAY_SIZE( inputs ), 300, outputs );


    geometry_msgs::Point p;
    visualization_msgs::Marker point;


    point.ns = "bezier";
    point.action =  visualization_msgs::Marker::ADD;
    point.pose.orientation.w = 1.0;

    point.id = 0;
    point.type = visualization_msgs::Marker::POINTS;

    //x and y width
    point.scale.x = 0.2;
    point.scale.y = 0.2;

    // Points are green
    point.color.g = 1.0f;
    point.color.a = 1.0;


    point.header.frame_id  = "map";
    point.header.stamp = ros::Time::now();

    //nav_msgs::Path path_map;



    for ( int x = 0; x < 500; x += 2 ) {


        /*
        p.x = outputs[x] + transform.getOrigin().x();
        p.y = outputs[x+1] + transform.getOrigin().y();
        p.z = 0 + transform.getOrigin().z();*/

        p.x = outputs[x] + transform.getOrigin().x();
        p.y = outputs[x+1] + transform.getOrigin().y();
        p.z = 0 + transform.getOrigin().z();

        //points.points.push_back(p);
        point.points.push_back(p);

}

    return point;
}



bool check_obstacles(nav_msgs::Path trajectory, nav_msgs::OccupancyGrid map_, geometry_msgs::Pose auxiliar_point[3], tf::StampedTransform transform)
{
    int width;
    int heigth;
    int index;
    float x_coordinate;
    float x_coordinate_1;
    float x_coordinate_2;
    float y_coordinate;
    float y_coordinate_1;
    float y_coordinate_2;
    float increment = 20.0;
    float tangent;
    float arc_tangent;
    int toggle=1;
    int aux_count = 0;


    geometry_msgs::Pose aux_pose;



  //for(trajectory_waypoint_cell){


    for(int i=0; i<1000; i++){ //tengo que ampliar a más iteraciones

       x_coordinate = trajectory.poses[i].pose.position.x;
       y_coordinate = trajectory.poses[i].pose.position.y;

       heigth = y_coordinate / map_.info.resolution;
       width = x_coordinate / map_.info.resolution;
       index = map_.info.width * heigth + width;
       //cout <<"El width es" << width<< endl;
       //cout <<"El index es" << index<< endl;
       //cout <<static_cast<int16_t>(map_.data[i]);


       if(static_cast<int16_t>(map_.data[index]) !=0){
           cout <<" There is an obstacle in " << index;
           cout <<" For the coordinates x " << trajectory.poses[i].pose.position.x;
           cout <<" For the coordinate y " << trajectory.poses[i].pose.position.y<<endl;

           //result = tan ( param * PI / 180.0 );

           x_coordinate_1 = trajectory.poses[i-1].pose.position.x;
           x_coordinate_2 = trajectory.poses[i+1].pose.position.x;
           y_coordinate_1 = trajectory.poses[i-1].pose.position.y;
           y_coordinate_2 = trajectory.poses[i+1].pose.position.y;


           tangent = ((y_coordinate_2 - y_coordinate_1)/(x_coordinate_2 - x_coordinate_1));
           arc_tangent = atan(tangent); //In radians

           do{
               //increment = increment +
           aux_pose.position.x = -1 *sin(arc_tangent) * increment + (x_coordinate_2 - x_coordinate_1)/2 + x_coordinate_1; //Tengo que sumar la posicion del punto intermedio
           aux_pose.position.y = -1 *cos(arc_tangent) * increment + (y_coordinate_2 - y_coordinate_1)/2 + y_coordinate_1;
           heigth = aux_pose.position.y / map_.info.resolution;
           width = aux_pose.position.x / map_.info.resolution;
           index = map_.info.width * heigth + width;
           toggle = toggle * -1;
           }

           while(static_cast<int16_t>(map_.data[index]) !=0);

           aux_pose.position.x = aux_pose.position.x - transform.getOrigin().x(); //Tengo que sumar la posicion del punto intermedio
           aux_pose.position.y = aux_pose.position.y - transform.getOrigin().y();


           auxiliar_point[aux_count] = aux_pose;
           cout <<"Valor de tangente" << tangent <<endl;
           cout <<"Valor de x1 " << x_coordinate_1 << endl;
           cout <<"Valor de y1 " << y_coordinate_1 << endl;
           cout <<"El punto auxiliar se encuentra en x " << auxiliar_point[aux_count].position.x + transform.getOrigin().x() << " en y " << auxiliar_point[aux_count].position.y + transform.getOrigin().y()<<endl;
           cout <<"El punto inicial es x "<< current_odom.pose.pose.position.x << " en y " << current_odom.pose.pose.position.y <<endl;
           cout <<"El punto final es x "<< goal_point.pose.position.x << " en y " << goal_point.pose.position.y<<endl;

           aux_count ++;
           return true;

       }
    }
}

/*void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map )
{
    int index = 0;
    float x = 0;
    float y = 0;
    int grid_x;
    int grid_y;

  //ROS_INFO("I heard: [%f]", map->data[0]);
    //ROS_INFO("El valor del output es: [%f]",  path_map.poses[20].pose.position.x); //uint32 width
    for (int i=0; i<1024; i++){
        x = path_map.poses[i].pose.position.x;
        y = path_map.poses[i].pose.position.y;

        grid_x = x / 0.158 + 1345;
        grid_y = y / 0.158 + 955;

        index  = (grid_y*2651 + grid_x);

        if(map->data[index] != 0){
            ROS_INFO("Hay un obstáculo en [%f]",  x, y);
        }
        //map->data[i,j]

    }


    /*
    for(int x=0; x<= map->info.width; x++){
        for(int y=0; y<= map->info.height; y++){
            index ++;
            if (x == x_goal && y == y_goal){
                map->data[index]

            }
        }
    }
    */
//}

int main(int argc, char **argv)
{


    double outputs[2048];
    // advanced curve
    double inputs[8] = {
        0, 0.0,
        0, 100.0,
        0, 195.0,
        0, 200.0
    };



   Bezier2D( inputs, ARRAY_SIZE( inputs ), 1024, outputs );

   /* for ( int x = 0; x < 2048; x += 2 ) {
        printf( "%li\t%li\n", (long)outputs[x], (long)outputs[x+1] );
    }*/


    //Hasta aquí el código pegado
    ros::init(argc, argv, "bezier");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<visualization_msgs::Marker>("chatter", 1000);
    ros::Publisher chatter_pub_path = nh.advertise<nav_msgs::Path>("chatter_path", 1000);
    ros::Publisher chatter_map_path = nh.advertise<nav_msgs::Path>("chatter_path_map", 1000);
    ros::Publisher chatter_map_path1 = nh.advertise<nav_msgs::Path>("chatter_path_map1", 1000);
    ros::Publisher chatter_map_path2 = nh.advertise<nav_msgs::Path>("chatter_path_map2", 1000);
    ros::Publisher chatter_map_array = nh.advertise<geometry_msgs::PoseArray>("chatter_map_array", 1000);
    ros::Publisher chatter_map_array1 = nh.advertise<geometry_msgs::PoseArray>("chatter_map_array1", 1000);
    ros::Publisher chatter_map_array2 = nh.advertise<geometry_msgs::PoseArray>("chatter_map_array2", 1000);
    //visualization_msgs::Marker

    ros::Publisher chatter_map_point = nh.advertise<visualization_msgs::Marker>("chatter_map_point", 1000);
    ros::Publisher chatter_map_point1 = nh.advertise<visualization_msgs::Marker>("chatter_map_point1", 1000);
    ros::Publisher chatter_map_point2 = nh.advertise<visualization_msgs::Marker>("chatter_map_point2", 1000);


    ros::Publisher check_pub =  nh.advertise<std_msgs::String>("check", 1000);

    //Lo añadido por Pablo
    ros::Subscriber map = nh.subscribe("map", 1000, mapCallback);
    /*
    ros::Subscriber goalPointCallback_sub = nh.subscribe("/move_base_simple/goal", 1000, goalPointCallback);
    ros::Subscriber current_odometry = nh.subscribe("current_odometry", 1000, odomCallback);
    */

    ros::Subscriber goalPointCallback_sub = nh.subscribe("goal_point", 1000, goalPointCallback);
    ros::Subscriber current_odometry = nh.subscribe("current_odom", 1000, odomCallback);


    //Delete when the right odom publisher has been set.

    current_odom.pose.pose.position.x = 0;
    current_odom.pose.pose.position.y = 0;
    goal_point.pose.position.x = 20;
    goal_point.pose.position.y = 15;

    current_odom1.pose.pose.position.x = 0;
    current_odom1.pose.pose.position.y = 0;
    goal_point1.pose.position.x = 10;
    goal_point1.pose.position.y = 5;

    current_odom2.pose.pose.position.x = 0;
    current_odom2.pose.pose.position.y = 0;
    goal_point2.pose.position.x = 20;
    goal_point2.pose.position.y = -10;



    ros::spinOnce(); //Llamo a los callback

    auxiliar_point[0].position.x = goal_point.pose.position.x;
    auxiliar_point[0].position.y = goal_point.pose.position.y;
    auxiliar_point[1].position.x = goal_point.pose.position.x;
    auxiliar_point[1].position.y = goal_point.pose.position.y;
    auxiliar_point[2].position.x = goal_point.pose.position.x;
    auxiliar_point[2].position.y = goal_point.pose.position.y;


    ros::Rate loop_rate(10);

    tf::TransformListener listener;
    //geometry_msgs::PoseArray auxiliar_point;

    //No lo entiendo
    bool obstacle_free = true;

    while (ros::ok())
    {
      ros::spinOnce(); //Llamo a los callback


      //TRANSFORMADA
      tf::StampedTransform transform;
       try{
         listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(50.0));
         listener.lookupTransform("/map", "/base_link",
                                  ros::Time(0), transform);
       }
       catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
       }

      if(!map_init) //&& goal_init
      {
            cout << "waiting goal or map" << endl;
      }
      else
      {
          // generate path
        trajectory = generate_path(goal_point, current_odom, auxiliar_point, transform); // current_odom.pose.pose.position.x
        trajectory1 = generate_path(goal_point1, current_odom1, auxiliar_point, transform);
        trajectory2 = generate_path(goal_point2, current_odom2, auxiliar_point, transform);


        trajectory_array = generate_array(goal_point, current_odom, auxiliar_point, transform);
        trajectory_array1 = generate_array(goal_point1, current_odom1, auxiliar_point, transform);
        trajectory_array2 = generate_array(goal_point2, current_odom2, auxiliar_point, transform);

        /*
        trajectory_point = generate_point(goal_point, current_odom, auxiliar_point, transform);
        trajectory_point1 = generate_point(goal_point1, current_odom1, auxiliar_point, transform);
        trajectory_point2 = generate_point(goal_point2, current_odom2, auxiliar_point, transform);
        */

        obstacle_free = check_obstacles(trajectory, map_, auxiliar_point, transform);  // map.info.width // map.info.height // map.info.resolution

        chatter_map_path.publish(trajectory);
        chatter_map_path1.publish(trajectory1);
        chatter_map_path2.publish(trajectory2);

        chatter_map_array.publish(trajectory_array);
        chatter_map_array1.publish(trajectory_array1);
        chatter_map_array2.publish(trajectory_array2);

        /*
        chatter_map_point.publish(trajectory_point);
        chatter_map_point1.publish(trajectory_point1);
        chatter_map_point2.publish(trajectory_point2);
        */

        //chatter_map_array.publish(trajectory);


        if (!obstacle_free) // trajectory not valid
        {
          // add new auxiliary point


        }
        else // trajectory valid
        {
          // publish trajectory
        }
      }


     /* int count =0;
      for(int i=0; i<(map_.info.height*map_.info.width); i++){
          cout <<  static_cast<int16_t>(map_.data[i]);
          count +=1;
          if(count ==100){
              cout <<endl;
              count = 0;
          }
      }*/


      loop_rate.sleep();

    //Imprimir el mapa por la terminal

    std::stringstream ss;
    std::stringstream check;
	
	//Defino points
	visualization_msgs::Marker points;
    //Defino path
    nav_msgs::Path path;
    //nav_msgs::Path path_map;



    points.header.frame_id  = "/base_link";
    points.header.stamp = ros::Time::now();
    points.ns = "bezier";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    //x and y width
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    path.header.frame_id = "base_link";
    path.header.stamp = ros::Time::now();

    //path_map.header.frame_id = "map";
    //path_map.header.stamp = ros::Time::now();



        //Me devuelve las x y las y. En forma de string
        std_msgs::String msgx;
        std_msgs::String msgy;

        //En forma de geometry_msgs
        geometry_msgs::Point p;
        geometry_msgs::PoseStamped pose;

        geometry_msgs::PoseStamped pose_map;


        /*
        p.x = (int32_t)i - 50;
        p.y = y;
        p.z = z;
        */


        for ( int x = 0; x < 2048; x += 2 ) {


            p.x = outputs[x];
            p.y = outputs[x+1];
            p.z = 0;
            double angle = 0;

            //Por cuaternios

            //double w = cos (0.5*angle*PI/180);
            //double x = p.x * sin (0.5*angle*PI/180);
            //double y = p.y * sin (0.5*angle*PI/180);
            //double z = p.z * sin (0.5*angle*PI/180);

            pose.pose.position.x = outputs[x];
            pose.pose.position.y = outputs[x+1];
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1.0; //Significa que no hay rotación.

            //Otro sistema de poses en referencia a /map
            pose_map.pose.position.x = pose.pose.position.x + transform.getOrigin().x();
            pose_map.pose.position.y = pose.pose.position.y + transform.getOrigin().y();
            pose_map.pose.position.z = pose.pose.position.z + transform.getOrigin().z();

            pose_map.pose.orientation.x = 0;
            pose_map.pose.orientation.y = 0;
            pose_map.pose.orientation.z = 0;
            pose_map.pose.orientation.w = 1.0;


            points.points.push_back(p);
            path.poses.push_back(pose);
            //path_map.poses.push_back(pose_map);

            x_transformada = pose_map.pose.position.x;
            y_transformada = pose_map.pose.position.y;


            //path.poses[].pose.position

            //chatter_pub.publish(msgy);
        }


        //marker_pub.publish(points);


        chatter_pub.publish(points);
        chatter_pub_path.publish(path);
        //chatter_map_path.publish(path_map);
        

        //marker_pub.publish(points);

    }

    return 0;
}
