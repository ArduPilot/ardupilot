// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------
// Makes sure cog is in the range 0-2pi
static float unwrap_2pi(float x)
{
   while (x < 0.0)  { x = x + 2*pi; } 
   while (x > 2*pi) { x = x - 2*pi; }
   return x; 
}
//-------------------------------------------------------------------------------
// Makes sure cog is in the range -pi to pi
float unwrap_pi(float x)
{
   while (x < -pi) { x = x + 2*pi; } 
   while (x >  pi) { x = x - 2*pi; }
   return x; 
}
//-------------------------------------------------------------------------------
struct CCD distance_and_bearing(struct wp_class wp1, struct wp_class wp2){
  // Calculate distance in [NM]
  float dist   = 0.0;
  float course = 0.0;
  float Radie = 6378.137; //  [km] WGS84
  //hal.console->printf("wp1 = %.5f  %.5f\n",wp1.lat,wp1.lon);
  //hal.console->printf("wp2 = %.5f  %.5f\n",wp2.lat,wp2.lon);

  if ((wp2.lat!=wp1.lat) || (wp2.lon!=wp1.lon)) {
     float dlat = (wp2.lat-wp1.lat); 
     if (abs(dlat)<0.000001){
        wp2.lat =wp2.lat + 0.000001;
        dlat = (wp2.lat-wp1.lat);
     }
     float dlon = wp2.lon-wp1.lon;
     float dfi  = log(tan(wp2.lat/2 + pi/4)/tan(wp1.lat/2 + pi/4));
     float q    = dlat/dfi;
     dist       = (float) (Radie/1.852*sqrt( dlat*dlat + q*q*dlon*dlon));
     course = (float) atan2( dlon, dfi);
     course = unwrap_2pi(course);
     //hal.console->printf("dist = %f     course = %f\n",dist, ToDeg(course));
  }
   CCD ccd={course,dist};
  return ccd;
}
//-------------------------------------------------------------------------------
wp_class get_closet_point_on_rhumbLine(struct wp_class A, struct wp_class B,  struct wp_class P){
    // Returns the point on line A-B that lies closest to P
    wp_class AP  = { P.lon - A.lon ,  P.lat - A.lat};
    wp_class AB  = { B.lon - A.lon ,  B.lat - A.lat};
    double ab2   = AB.lon*AB.lon + AB.lat*AB.lat;
    double ap_ab = AP.lon*AB.lon + AP.lat*AB.lat;
    double t     = ap_ab / ab2;
    if (t < 0.0) {t = 0.0;} // Takes care of overshoot at A
    if (t > 1.0) {t = 1.0;} // Takes care of overshoot at B
    wp_class closest_point = {A.lon + AB.lon * t , A.lat + AB.lat * t};
    //hal.console->printf("Closest point on rhumb: lon=%f,    lat = %.5f \n",ToDeg(closest_point.lon),ToDeg(closest_point.lat));
    return closest_point;
}
//-------------------------------------------------------------------------------
float calc_course_to_wp_wrpt_Xtrack_error(struct wp_class wpA, struct wp_class wpB, struct wp_class pos) {
   // Calcs cc to stear with respect to rhumb line correcton.
   // Rhumb line is from wpA -> wpB
   // if k_xtrack = 0.0 : return course to wpB
   // if k_xtrack = 1.0 : return course straigth to nearest pont on rhumb-line
   wp_class wpC;
   wp_class wpD;

   if (k_xtrack<0.01) {
      wpD = wpB; // Point to steer towards
   }
   else{
      wpC = get_closet_point_on_rhumbLine(wpA,wpB, pos);        // Closest point on Rhumb-line
      wpD.lon = wpB.lon*(1-k_xtrack)+wpC.lon*k_xtrack;
      wpD.lat = wpB.lat*(1-k_xtrack)+wpC.lat*k_xtrack; // Point to steer towards
   }
   CCD   tmp = distance_and_bearing(pos, wpD);  // Returns both course and distance
   float course = unwrap_2pi(tmp.course);
   //hal.console->printf("Steer = %f deg\n",ToDeg(course));
   return course;
}
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
void set_next_WP(Location wp)
{
    //prev_wp = next_wp;
    //next_wp = wp;
}
//-------------------------------------------------------------------------------
 // waypoint wp1; wp1.lon=ToRad(10); wp1.lat=ToRad(50);
 // waypoint wp2; wp2.lon=ToRad(20); wp2.lat=ToRad(50);
    
  //   CCD tmp = distance_and_bearing(wp1, wp2);
  //   float course = tmp.course;
  //   float dist = tmp.dist;

   //  waypoint P= { ToRad(15),ToRad(60)};
    //  float cc = calc_course_to_wp_wrpt_Xtrack_error( wp1, wp2, P, 0.5);
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------


