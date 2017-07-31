/*

this library collects points of drone position, records them in memory and eliminates 
unnecessary data - redundant points and loops, so it always knows 
the shortest way to home via visited points.
 
 Copyright night_ghost@ykoctpa.ru 2017
 
 license GPLV3 or above
   
*/

#include "AP_WayBack.h"

extern const AP_HAL::HAL& hal;

AP_WayBack::Point* AP_WayBack::points;
uint16_t          AP_WayBack::max_num_points=0; // size of storage
uint16_t          AP_WayBack::num_points=0;          // all points count
uint16_t          AP_WayBack::points_count=0;        // good points count
uint16_t          AP_WayBack::new_points=0;          // number of  points after last reduce

uint16_t          AP_WayBack::last_loop_check=2;     // last leg checked for loop
uint16_t          AP_WayBack::last_reduce=0;         // last leg checked for reduce
uint16_t          AP_WayBack::last_big_reduce=0;     // last leg checked when new point added
uint16_t          AP_WayBack::last_raw_point=0;   

bool              AP_WayBack::in_loop_reduce = false;
uint16_t          AP_WayBack::hi_loop_border;

bool              AP_WayBack::recording = false;
float             AP_WayBack::_epsilon; 
uint32_t          AP_WayBack::last_point_time=0;

uint16_t          AP_WayBack::loop_leg_low=0; // lowest checked - we move back!
uint16_t          AP_WayBack::loop_leg_high=0;// higest checked - will be next last_loop_check on finish
uint16_t          AP_WayBack::loop_leg_ptr=0;//  current, goes to loop_leg_low

#if defined(WAYBACK_DEBUG)
bool              AP_WayBack::_debug_mode = false;
#endif

bool              AP_WayBack::initialized=false;

const AP_Param::GroupInfo AP_WayBack::var_info[] = {
    // @Param: _USE
    // @DisplayName: AP_Wayback use
    // @Description: 0 to not use, !0 to use
    // @Values: 0:not use, 1:use
    // @User: Standard
    AP_GROUPINFO("_USE",  0, AP_WayBack, _params.use,       0), 

    // @Param: _EPS_DISTANCE
    // @DisplayName: Eps distance (in meters)
    // @Description: Minimal distance between points, in meters
    // @User: Standard
    AP_GROUPINFO("_EPS_DISTANCE",  1, AP_WayBack, _params.eps,      EPS_DISTANCE),

    // @Param: _BLIND_SHORTCUT
    // @DisplayName: Enable blind shortcuts
    // @Description: If enabled, points on track wich are near to each other will be considered as intersection
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_BLIND_SHORTCUT",  2, AP_WayBack, _params.blind_shortcut,       1),

    AP_GROUPEND
};

struct AP_WayBack::Params AP_WayBack::_params;

/*
 * init - perform required initialisation
 */
void AP_WayBack::init()
{    
    if(initialized || !_params.use) return; // 2nd call not cause new io_process
    
    _epsilon = TRACK_EPS; // initial track error
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_WayBack::tick, void));
    initialized=true;
}


bool AP_WayBack::start()
{
    if(!initialized) init();
    if(points==nullptr) {
        if(_params.use) {
            uint32_t sz = sizeof(Point) * NUM_TRACK_POINTS;
            while(1){
                points = (Point *)malloc(sz); // try to allocate buffer
                if(points!=nullptr) {
                    max_num_points = sz / sizeof(Point); // calculate size in points
                    break;
                }
                sz /= 2; // no memory - try to reduce number of points
                if(sz==0) return false; // no memory at all
            }
        }
        
        num_points=0;
        points_count=0;
        last_loop_check=2; // last leg checked for loop
        last_reduce=0;     // last leg checked for reduce

        last_point_time=0; // definitely less than current millis()
        last_big_reduce=0;
        last_raw_point=0;   
    }
    
    
    recording=true;    
    return true;
}

void AP_WayBack::stop()
{
    recording=false;
    
    if(num_points) num_points -= 1; // skip the last point - current coordinates

    simplify(last_reduce, num_points); // reduce all remaining points

    // remove last loops
    uint16_t p1=points_count-1; // last good point
    
    while(last_loop_check > p1){
        uint16_t p0 = move_back(p1,1); // points can be not removed so we use relative coordinates
         // check segment for track intersection

    DBG_PRINT("check loop1 ");     DBG_PRINTVAR(p0);    DBG_PRINTVARLN(p1);

        uint16_t sb = try_remove_loop(p0, p1);

        if(sb) { // loop found
            p0=move_back(sb,1);    // prev point 
    DBG_PRINT("loop found at ");       DBG_PRINTVARLN(p1);            

            simplify(p0, p1); // try to reduce after removal
            break; // we move back so just find shortest path
        }
        p1 = p0; // set left point as right    
    }

}

// get last point and remove it from track
bool AP_WayBack::get_point(float &x, float &y) 
{
//    if(recording) stop(); Is we need it?
    
    for(uint16_t i=num_points;i>0;i--){ 
        uint16_t idx = i-1;
        if(is_good(points[idx])){ // found a good point
            num_points=idx; // remove last
            
            x=points[idx].x;
            y=points[idx].y;
            return true;
        }
    }
    return false; // no more points so go directly to HOME point
}

/*
 * tick - main call to get data from AHRS
 */
void AP_WayBack::tick(void)
{ 

    if(points==NULL) return;

    uint32_t now = AP_HAL::millis();
    uint32_t dt  = now - last_point_time;
    if( dt < 1000) {    // 1 point per second
        yield(); // don't disturbe for needed time
        return; 
    }

#if defined(WAYBACK_DEBUG)
    if(_debug_mode) return; // work in soft emulation
#endif

    if(recording){

// here we should to get a current UAV location filtered with EKF to prevent recording of GPS glitches
#if FRAME_CONFIG ==  MULTICOPTER_FRAME
        Vector3f pos;
        _ahrs.get_relative_position_NED_home(pos);
        add_point(pos.x, pos.y);

#else
        Location loc;
        if (!_ahrs.get_position(loc)) {
            return;
        }
        add_point(loc.lat, loc.lng);
#endif
    }

}



#if defined(WAYBACK_DEBUG)
// print out track points
bool AP_WayBack::show_track(uint16_t &i, float &x, float &y ) 
{
    while(i<num_points){ 
        if(is_good(points[i])){ // found a good point
            x=points[i].x;
            y=points[i].y;
            i++;
            return true;
        }
        i++;
    }
    return false;
}
#endif



// private 

// move back on STEPS good points
uint16_t AP_WayBack::move_back(uint16_t from, uint16_t steps)
{
    if(from==0) return 0;
    for(uint16_t i=from-1;i>0;i--){ 
        if(is_good(points[i])){ // found a good point
            if(--steps == 0) return i;
        }
    }

    return 0; //failed
}


// move forward on STEPS good points
uint16_t AP_WayBack::move_forw(uint16_t from, uint16_t steps)
{
    if(from+1>=num_points)  return from;
    
    for(uint16_t i=from+1;i<num_points;i++){ 
        if(is_good(points[i])){ // found a good point
            if(--steps == 0) return i;
            from=i; // remember last good
        }
    }

    return from; //failed, return last good
}

void AP_WayBack::add_point(float x, float y)
{
    bool was_reduce=false;
    uint16_t p0;       // leg begin
    uint16_t p1;       // leg end

    if(dist(x,y,points[num_points-1]) > _epsilon) { // we can add a point

        Point &p = points[num_points];

        p.x=x;
        p.y=y;
        do_good(p);
    
        num_points++;   // increment index
        points_count++; // and number of good points
        new_points++;
    
         //don't mess to loops     we can simplify                                                                   or  we MUST simplify
        if(!in_loop_reduce && (new_points > MIN_SIMPLIFY_POINTS && num_points > (MIN_SIMPLIFY_POINTS+RAW_POINTS)) || num_points>=max_num_points) { // simplify if we have enough points or have no room
again:
            new_points=0;
        
            uint16_t np=points_count; // remember number of good points
        
            if(num_points==max_num_points) { // just written last track point. This shouldn't ever be but...
                p0 = 0;           // from beginning
                p1 = num_points;  // and to end
                _epsilon *=2;     // twice error and simplify all track
            
                last_reduce =1;  // that numbers are wrong now
                last_loop_check /=2;      if(last_loop_check<2) last_loop_check=2;
            } else {
                p0 = last_reduce; // this is last optimized point
                p1 = move_back(num_points, RAW_POINTS); // skip points that should be raw        
            }

    DBG_PRINT("reduce ");    DBG_PRINTVAR(p0);     DBG_PRINTVARLN(p1);

            was_reduce = simplify(p0, p1);
        
            uint16_t removed = np-points_count; // number of removed points
            uint16_t left = (p1-last_raw_point);
            if(left > removed) left-=removed;
            else               left=0;
            
//            if(was_reduce) last_point=p1; 
                            
            if(p1 > removed) p1-=removed; // we will remove points before next step! so convert to absolute index
            hi_loop_border = p1;          // check for loops up to here
            last_raw_point = p1;

            last_loop_check = p0; // check new segments for intersections

            if(left >1) {  // осталась одна новая точка 
                last_reduce = move_back(last_big_reduce, SIMPLIFY_MOVE_BACK); // move back by SIMPLIFY_MOVE_BACK points
                last_big_reduce = p1;
                DBG_PRINTVAR( left); DBG_PRINTVARLN( last_reduce);

                in_loop_reduce=true;   // check newly formed segments
            } else { // no new points
                last_reduce = p1; // just from last point
            }


    DBG_PRINTVARLN( left);

    DBG_PRINTVAR( removed);   DBG_PRINTVAR( last_reduce);  DBG_PRINTVARLN( last_big_reduce);
        
            if((num_points-removed)>=max_num_points) goto again; // we still don't have a room for new point
        }
    }
    
    // if enabled or we should to clear room for new points: we check one leg per point so number of free points should be more than not checked legs
    if(! was_reduce ) {
    
        // to exclude quadratic complexity we check intersection only one step in time
        if(in_loop_reduce){      //  уже обрабатываем или отстали на нужное количество точек 

            // check one leg from last_loop_check
            p0 = last_loop_check;
            p1 = move_forw(p0,1); // points can be not removed so we use relative coordinates

         // check segment for track intersection
            uint16_t sb=0;
            
            if(p1>p0){
    DBG_PRINT("check loop ");     DBG_PRINTVAR(p0);    DBG_PRINTVARLN(p1);

                sb = try_remove_loop(p0, p1);
            }


            if(sb) { // loop found
                p1=sb-1;    // prev point as last checked

                last_reduce=p1; // try to reduce from here
                new_points +=  MIN_SIMPLIFY_POINTS; // force reduce on next point
            
                if(p1<2) p1=2; // but not earlier than 2
            
                was_reduce = true; // points was removed
            } else {
                if(p1 >= hi_loop_border)  in_loop_reduce=false;
            }
 
            
            last_loop_check = p1;                    
        }
    }
    // do it only if was any reductions
    if(was_reduce) {
        squizze();      // remove bad points
    }

}




/*
   Determine the intersection point of two line segments (Paul Bourke)
   Return FALSE if the lines don't intersect
*/
bool AP_WayBack::linesIntersect(const Point &p1, const Point &p2,
                                const Point &p3, const Point &p4,
                                Point *cross) // coordinates of intersection
{
   float denom,numera,numerb;

   denom  = (p4.y-p3.y) * (p2.x-p1.x) - (p4.x-p3.x) * (p2.y-p1.y);
   numera = (p4.x-p3.x) * (p1.y-p3.y) - (p4.y-p3.y) * (p1.x-p3.x);
   numerb = (p2.x-p1.x) * (p1.y-p3.y) - (p2.y-p1.y) * (p1.x-p3.x);

   // Are the line coincident? 
   if (is_equal(numera,0.0)  && is_equal(numerb, 0.0)  && is_equal(denom,0.0) ) {
      cross->x = (p1.x + p2.x) / 2;
      cross->y = (p1.y + p2.y) / 2;
      return true;
   }

   // Are the line parallel 
   if (is_equal(denom, 0.0)) {
      return false;
   }

   float mua,mub;
   // Is the intersection along the the segments 
   mua = numera / denom;
   mub = numerb / denom;
   if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
      return false;
   }
   cross->x = p1.x + mua * (p2.x - p1.x);
   cross->y = p1.y + mua * (p2.y - p1.y);
   return true;
}


// full distance with bounds check
float AP_WayBack::distanceToSegment(const Point &p, 
                                    const Point &p1, const Point &p2,
                                    Point *closest) // return coordinates of closest point
{
    float diffX = p2.x - p1.x;
    float diffY = p2.y - p1.y;

    Point n = p1;

    if ( !(is_zero(diffX) && is_zero(diffY)) ) {

        float t = ((p.x - p1.x) * diffX + (p.y - p1.y) * diffY) / (diffX * diffX + diffY * diffY);

        if (t < 0) {           //point is nearest to the first point i.e x1 and y1
        //    n = p1;
        } else if (t > 1) {    //point is nearest to the end point i.e x2 and y2
            n = p2;
        } else {               //if perpendicular line intersect the line segment.
            n.x = (p1.x + t * diffX);
            n.y = (p1.y + t * diffY);
        }
    }
    
    *closest=n;// closest point

    return dist(p,n);     //returning shortest distance
}



// check if lines are close enough to treat as intersecting
uint8_t AP_WayBack::linesAreClose(const Point &p1, const Point &p2,
                                  const Point &p3, const Point &p4,
                                  Point *closest,
                                  uint16_t *np)
{
    // check distance from both points of one segment to 2nd segment, and vice versa
    
    float d;
    uint8_t seg=1;
    uint16_t n;
    
    float eps = _epsilon*SHORTCUT_FACTOR;

    n=1;               // 1st point of 2nd segment  1st segment as line

    d=distanceToSegment(p3, p1, p2, closest);
    if(d<eps) goto found;

    n=2;              // 2nd point of 2nd segment

    d=distanceToSegment(p4, p1, p2, closest);
    if(d<eps) goto found;


    n=1;              // 1st point of 1st segment  2nd segment as line
    seg=2;

    d=distanceToSegment(p1, p3, p4, closest);
    if(d<eps) goto found;

    n=2;               // 2nd point of 1st segment
    
    d=distanceToSegment(p2, p3, p4, closest);
    if(d<eps) goto found;

    return 0;
    
found:
//    DBG_PRINT("closest found ");  DBG_PRINTVAR(closest->x); DBG_PRINTVAR(closest->y); DBG_PRINTVARLN(d); 

    *np = n;
    return seg;
}

uint16_t AP_WayBack::try_remove_loop(uint16_t sb, uint16_t se) // begin and end of checking segment
{
    if(!(se>sb) ) return 0;
    
    uint8_t  sect_count=0; // counts intersecions

    uint16_t s0 = 0; // 1st point of segment
    uint16_t s_last=0;
    
    for(uint16_t i=1; i<num_points && i<sb; i++) { // stop when reached tested leg
// we not check loops when have bad points!        if(!is_good(points[i])) continue; // skip bad points 
        
        // now we have a segment s0..i so check intersection
        Point p;
        
        if(linesIntersect(points[s0], points[i], // 1st and 2nd points of 1st segment
                          points[sb], points[se],// 1st and 2nd points of 2nd segment
                          &p )) // intersection
        { // YES it intersects!
            do_good(p);
        
            points[i]=p;        // replace 2nd point of 1st segment by point of intersection        
            removePoints(i+1,se); // remove all points up to 2nd point of 2nd segment

    DBG_PRINT("loop found at ");       DBG_PRINTVARLN(i);
            return i;  // point of intersection
        
        } else if(_params.blind_shortcut) { // try to treat close points as intersecting
            uint16_t n;

            // надо исключить случай когда точки лежат на соседних сегментах
            if( i+1 >= sb )  goto skip_it; // наехали на конец проверки
            if(s_last == s0) goto skip_it; // самое начало
        
        // проверяем предыдущий отрезок - а то может быть пересечение только будет а мы уже отметили близкую точку.
            uint8_t seg=linesAreClose(points[s_last], points[s0],// 1st and 2nd points of 1st segment
                                      points[sb],     points[se], // 1st and 2nd points of 2nd segment
                                      &p, &n );
            if(seg!=0){  // yes!                        
                do_good(p);

                if(seg==1) { // point on 1st segment 
            
                    uint16_t ep;
                    if(n==1) ep=sb;
                    else     ep=se;

                    // replace 2nd point of 1st segment by point of intersection
                    points[i]=p;
                    removePoints(i+1,ep); // remove all points up to 2nd point of 2nd segment

            DBG_PRINT("close to 1seg at ");       DBG_PRINTVARLN(i);
                    return i;  // point of intersection

                }else {  // point on 2nd segment
                    uint16_t sp;
                    if(n==1 && s_last!=0) sp=s_last;
                    else                  sp=s0;

                    points[sb]=p;       // replace 1st point of 2nd segment with intersection
                    removePoints(sp+1, sb); // remove all points up to 2nd point of 2nd segment

            DBG_PRINT("close to 2seg at ");       DBG_PRINTVARLN(sp);
                    return sp;  // point of intersection
                }
            }
    
        }
skip_it:
        // move to next segment
        s_last = s0; // предыдущий отрезок
        s0 = i;
    
        yield(); // segment a time - give a chance to another tasks            
    }
    return sect_count;

}


#if 0 // line-line distance in 3D

/*
   Calculate the line segment PaPb that is the shortest route between
   two lines P1P2 and P3P4. Calculate also the values of mua and mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Return FALSE if no solution exists.
*/
bool lineLineIntersect(const Point &p1, const Point &p2, // 1st segment
                       const Point &p3, const Point &p4, // 2nd segment
                       Point *pa, Point *pb, // resulting shortest segment
                       float *mua, float *mub) // koefficients
{
   XYZ p13,p43,p21;
   float d1343,d4321,d1321,d4343,d2121;
   float numer,denom;

   p13.x = p1.x - p3.x;
   p13.y = p1.y - p3.y;
   p13.z = p1.z - p3.z;
   p43.x = p4.x - p3.x;
   p43.y = p4.y - p3.y;
   p43.z = p4.z - p3.z;
   if (ABS(p43.x) < EPS && ABS(p43.y) < EPS && ABS(p43.z) < EPS)
      return false;
      
   p21.x = p2.x - p1.x;
   p21.y = p2.y - p1.y;
   p21.z = p2.z - p1.z;
   if (ABS(p21.x) < EPS && ABS(p21.y) < EPS && ABS(p21.z) < EPS)
      return false;

   d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
   d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
   d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
   d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
   d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

   denom = d2121 * d4343 - d4321 * d4321;
   if (ABS(denom) < EPS)
      return false;
      
   numer = d1343 * d4321 - d1321 * d4343;

   *mua = numer / denom;
   *mub = (d1343 + d4321 * (*mua)) / d4343;

   pa->x = p1.x + *mua * p21.x;
   pa->y = p1.y + *mua * p21.y;
   pa->z = p1.z + *mua * p21.z;
   pb->x = p3.x + *mub * p43.x;
   pb->y = p3.y + *mub * p43.y;
   pb->z = p3.z + *mub * p43.z;

    // length of (pa,pb) is a distance

   return true;
}
#endif

float AP_WayBack::dist( float p1X, float p1Y, float p2X, float p2Y)
{
    p1X -= p2X;
    p1Y -= p2Y;
    return sqrt( p1X*p1X + p1Y*p1Y);
}


// distance from p to line [p1,p2]  without check for segment bounds
float AP_WayBack::find_perpendicular_distance(const Point &p, const Point &p1, const Point &p2)
{
	float slope, intercept;

	if (is_zero(p1.x - p2.x)) {
	    return fabs(p.x - p1.x); // segment's length==0
	} else {
	    slope = (p2.y - p1.y) / (p2.x - p1.x); // segment's slope
	    intercept = p1.y - (slope * p1.x);
	    return fabs(slope * p.x - p.y + intercept) / sqrt(slope*slope + 1);
	}
}

/* any-dimension case but much less effective

// dist_Point_to_Line(): get the distance of a point to a line
//     Input:  a Point P and a Line L (in any dimension)
//     Return: the shortest distance from P to L

// Line with defining endpoints {Point P0, P1;}

// dot product (3D) which allows vector operations in arguments
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
#define norm(v)     sqrt(dot(v,v))     // norm = length of  vector
#define d(u,v)      norm(u-v)          // distance = norm of difference

float dist_Point_to_Line(const Point &p, const Point &p1, const Point p2)
{
     Vector v = p2 - p1;
     Vector w = p - p2;

     double c1 = dot(w,v);
     double c2 = dot(v,v);
     double b = c1 / c2;

     Point pb = p1 + b * v;
     return d(p, pb);
}

*/


// see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.95.5882&rep=rep1&type=pdf

bool AP_WayBack::simplify( uint16_t start, uint16_t end)
{
    return rdp_simplify( start, end);
//    return reumannWitkam_simplify( start, end);
}



// Simplifies a 2D dimensional line according to the Ramer-Douglas-Peucker algorithm
bool AP_WayBack::rdp_simplify( uint16_t start, uint16_t end)
{
    if(start == end) return false;
        Point &startP = points[start];
        Point &endP   = points[end];
    
	uint16_t   index = -1;
	float dist  = 0.0, current_dist;
	uint16_t    i;
	bool ret=false;

	if (end - start < 2) {
		return ret;
	}

	for (i = start + 1; i < end; i++) {
	        Point &p = points[i];
		if (!is_good(p)) // skip removed points
			continue;

		current_dist = find_perpendicular_distance(p, startP, endP);

		if (current_dist > dist) {
			dist = current_dist;
			index = i;
		}
	}
        yield(); // we consume a lot of time so give a chance to another tasks

	if (dist > _epsilon) {
		ret = rdp_simplify(start, index);
		ret = rdp_simplify(index, end) || ret;
		return ret;
	} else {
	        removePoints(start + 1, end);
		return true;
	}
	
}


bool AP_WayBack::reumannWitkam_simplify(uint16_t key, uint16_t end)
{
    bool ret=false;
  
    while(key+3 < end){ 

        uint16_t test= key+2;
        while (test < end) {
            float dist= find_perpendicular_distance(points[test], points[key], points[key+1]);

    DBG_PRINTVAR(dist); DBG_PRINTVARLN(test);
            
            if( dist > _epsilon) break;
            test++;
        }
        ret=removePoints(key+1, test-1);

        key++;
    }
    return ret;
}


bool AP_WayBack::removePoints(uint16_t start, uint16_t end)
{
    bool ret=false;

    if(start>=end) return ret; // to remove empty messages

DBG_PRINT("remove poins "); DBG_PRINTVARLN(start); DBG_PRINTVARLN(end);

    while(start<end){
        ret=true;
        if(is_good(points[start])){
            do_bad(points[start]);  // remove all points in gap
            points_count--;
        }
        start++;
    }
    return ret;
}

// remove unused points from points array
void AP_WayBack::squizze(){ 
    uint16_t wp=0;
    uint16_t rp=0;
    bool gap_found=false;
    
    for(uint16_t i=0;i<num_points;i++){
        if(!gap_found){ // search for empty slot
            if(!is_good(points[i])){
                wp=i;
                gap_found=true;
            }
        } else { //search for good point    
            if(is_good(points[i])){
                rp=i;
                break; // got both points
            }
        }
    }
    
    if(!gap_found) return; // в массиве нет дыр
    
    yield(); // give current tick to another tasks

DBG_PRINT("squizze"); DBG_PRINTVAR(wp); DBG_PRINTVAR(rp); DBG_PRINTVARLN(num_points);


    for(;rp<num_points; rp++){          // scan for points
        if(is_good(points[rp])) { // and write sequentally a good ones
            points[wp++]=points[rp];
            do_bad(points[rp]); // moved - for debug only 
        }
    }
    num_points = wp;
    points_count = num_points;  // all points are good

DBG_PRINT("new");  DBG_PRINTVARLN(num_points);

}



#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
 // no threads but just cooperative multitask
#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include <AP_HAL_REVOMINI/Scheduler.h>
using namespace REVOMINI;
void AP_WayBack::yield() 
{
    REVOMINIScheduler::yield(0); // give other tasks a chance do run
}

#else // we don't should do anything if HAL has a POSIX threads

void AP_WayBack::yield(){}
#endif

