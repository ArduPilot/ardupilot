/*
 * File:
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <ode/ode.h>
#include <plugins/physics.h>

dBodyID vehicleID; 

static pthread_mutex_t mutex; // needed to run with multi-threaded version of ODE

/*
 * Note: This plugin will become operational only after it was compiled and associated with the current world (.wbt).
 * To associate this plugin with the world follow these steps:
 *  1. In the Scene Tree, expand the "WorldInfo" node and select its "physics" field
 *  2. Then hit the [Select] button at the bottom of the Scene Tree
 *  3. In the list choose the name of this plugin (same as this file without the extention)
 *  4. Then save the .wbt by hitting the "Save" button in the toolbar of the 3D view
 *  5. Then reload the world: the plugin should now load and execute with the current simulation
 */

void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);
  
  vehicleID = dWebotsGetBodyFromDEF("DEF_VEHICLE");
  if (vehicleID == NULL)
  {
     dWebotsConsolePrintf("WARNING: NO DEF_VEHICLE OBJECT TO CONTROL no wind calculations will be applied.\n");
     return ;
  }
  else
  {
    dWebotsConsolePrintf("DEF_VEHICLE object found.\n");
     
  }
  /*
   * Get ODE object from the .wbt model, e.g.
   *   dBodyID body1 = dWebotsGetBodyFromDEF("MY_ROBOT");
   *   dBodyID body2 = dWebotsGetBodyFromDEF("MY_SOLID");
   *   dGeomID geom2 = dWebotsGetGeomFromDEF("MY_SOLID");
   * If an object is not found in the .wbt world, the function returns NULL.
   * Your code should correcly handle the NULL cases because otherwise a segmentation fault will crash Webots.
   *
   * This function is also often used to add joints to the simulation, e.g.
   *   dWorldID world = dBodyGetWorld(body1);
   *   pthread_mutex_lock(&mutex);
   *   dJointID joint = dJointCreateBall(world, 0);
   *   dJointAttach(joint, body1, body2);
   *   pthread_mutex_unlock(&mutex);
   *   ...
   */
}

void webots_physics_step() {
  /*
   * Do here what needs to be done at every time step, e.g. add forces to bodies
   *   dBodyAddForce(body1, f[0], f[1], f[2]);
   *   ...
   */
        
   int dataSize;
   if (vehicleID != NULL)
   {
      const char *data = (const char *)dWebotsReceive(&dataSize);
      if (dataSize > 0)
      {
        float wind[4];
        memcpy (&wind[0],data,sizeof(float)*4);
#if 0        
        dWebotsConsolePrintf("wind: %f %f %f %f \n", wind[0],wind[1],wind[2],wind[3]);
#endif        
        dBodyAddForce(vehicleID, wind[1], wind[2], wind[3]);
      }
     
   }
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  /*
   * This function needs to be implemented if you want to overide Webots collision detection.
   * It must return 1 if the collision was handled and 0 otherwise.
   * Note that contact joints should be added to the contact_joint_group which can change over the time, e.g.
   *   n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
   *   dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
   *   dWorldID world = dBodyGetWorld(body1);
   *   ...
   *   pthread_mutex_lock(&mutex);
   *   dJointCreateContact(world, contact_joint_group, &contact[i])
   *   dJointAttach(contact_joint, body1, body2);
   *   pthread_mutex_unlock(&mutex);
   *   ...
   */
  return 0;
}

void webots_physics_cleanup() {
  /*
   * Here you need to free any memory you allocated in above, close files, etc.
   * You do not need to free any ODE object, they will be freed by Webots.
   */
  pthread_mutex_destroy(&mutex);
}
