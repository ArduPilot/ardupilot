#ifndef _RALPHIE_STATE_TASK_H_
#define _RALPHIE_STATE_TASK_H_

#include "../../Plane.h"

class LqtStateManager {

	

public:
	/* Execute the task */
	void executeUpdateStateTask();

	/* Access to current aircraft state */
	void currentState();

	/* Access to current desired state */
	void desiredState();

};


#endif /* RALPHIE_STATE_TASK_H_ */

