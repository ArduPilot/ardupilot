/*
  SITL handling

  This simulates a CAN Peripheral on a SLCAN port

  Siddharth Bharat Purohit
 */


#include <AP_UAVCAN/AP_UAVCAN_SLCAN.h>
#include "UARTDriver.h"
#include "SITL_State.h"

using namespace HALSITL;
extern const AP_HAL::HAL& hal;
#pragma GCC diagnostic ignored "-Wunused-result"

int SITL_State::CANDriver::self_write_fd[2] = {-1,-1};
int SITL_State::CANDriver::self_read_fd[2] = {-1,-1};
int SITL_State::CANDriver::client_write_fd[2] = {-1,-1};
int SITL_State::CANDriver::client_read_fd[2] = {-1,-1};
HAL_Semaphore SITL_State::CANDriver::spin_sem;
bool SITL_State::CANDriver::reader_thread_initialised;

//SLCAN handle methods
bool SITL_State::CANDriver::begin(uint32_t bitrate, uint8_t can_number)
{
    if (driver_.init(bitrate, SLCAN::CAN::NormalMode, self_read_fd[can_number], self_write_fd[can_number]) < 0) {
        return false;
    }
    _can_number = can_number;

    if (!reader_thread_initialised) {
        if (!hal.scheduler->thread_create(FUNCTOR_BIND(_sitlState, &SITL_State::can_reader, void), "SITLSLCAN", 4096, AP_HAL::Scheduler::PRIORITY_CAN, -1)) {
            return false;
        }
        reader_thread_initialised = true;
    }
    initialized(true);
    return true;
}


bool SITL_State::CANDriver::is_initialized()
{
    return initialized_;
}

void SITL_State::CANDriver::initialized(bool val)
{
    initialized_ = val;
}

uavcan::CanSelectMasks SITL_State::CANDriver::makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces])
{
    uavcan::CanSelectMasks msk;

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (!driver_.is_initialized()) {
            continue;
        }

        if (!driver_.isRxBufferEmpty()) {
            msk.read |= 1 << i;
        }

        if (pending_tx[i] != nullptr) {
            if (driver_.canAcceptNewTxFrame()) {
                msk.write |= 1 << i;
            }
        }
    }

    return msk;
}

int16_t SITL_State::CANDriver::select(uavcan::CanSelectMasks& inout_masks,
                                  const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());

    inout_masks = makeSelectMasks(pending_tx); // Check if we already have some of the requested events
    if ((inout_masks.read & in_masks.read) != 0 || (inout_masks.write & in_masks.write) != 0) {
        return 1;
    }

    _irq_handler_ctx = pthread_self();

    if (blocking_deadline.toUSec()) {
        struct timespec req;
#if defined(__APPLE__)
        req.tv_sec =  (time_t)((blocking_deadline - time).toUSec()/1000000);
        req.tv_nsec = ((blocking_deadline - time).toUSec() % 1000000) * 1000;
        pthread_cond_timedwait_relative_np(&_irq_handler_cond, &_irq_handler_mtx, &req);
#else
        pthread_condattr_t attr;
        pthread_condattr_init( &attr);
        pthread_condattr_setclock( &attr, CLOCK_MONOTONIC);
        pthread_cond_init( &_irq_handler_cond, &attr);
        clock_gettime(CLOCK_MONOTONIC, &req);
        pthread_mutex_lock(&_irq_handler_mtx);
        req.tv_sec +=  (time_t)((blocking_deadline - time).toUSec()/1000000);
        req.tv_nsec += ((blocking_deadline - time).toUSec() % 1000000) * 1000;
        pthread_cond_timedwait(&_irq_handler_cond, &_irq_handler_mtx, &req);
#endif
        pthread_mutex_unlock(&_irq_handler_mtx);
    }
    inout_masks = makeSelectMasks(pending_tx); // Return what we got even if none of the requested events are set
    return 1; // Return value doesn't matter as long as it is non-negative
}


int read_by_fd(int _read_fd, const uint8_t *buf, uint16_t count)
{
#ifdef FIONREAD
    // use FIONREAD to get exact value if possible
    int num_ready;
    while (ioctl(_read_fd, FIONREAD, &num_ready) == 0 && num_ready > 3000) {
        // the pipe is filling up - drain it
        uint8_t tmp[128];
        if (read(_read_fd, tmp, sizeof(tmp)) != sizeof(tmp)) {
            break;
        }
    }
#endif
    return read(_read_fd, (void*)buf, count);
}

void SITL_State::can_reader(void)
{
    while (true) {
        //Read buffer recursively until both managers' interfaces' i/p buffers are empty
        while (true) {
            uint8_t data[2];
            int ret1 = read_by_fd(SITL_State::CANDriver::self_read_fd[0], &data[0], 1);
            int ret2 = read_by_fd(SITL_State::CANDriver::self_read_fd[1], &data[1], 1);
            if (ret1 == -1 && ret2 == -1) {
                break;
            }
            std::list<SITL_State::CANDriver*>::iterator can_mgr;
            for(can_mgr=_can_mgr.begin();can_mgr!=_can_mgr.end();can_mgr++) {
                if (*can_mgr == nullptr) {
                    continue;
                }
                (*can_mgr)->driver_.addByte(data[(*can_mgr)->_can_number]);
            }
        }
        // Signal respective threads to continue processing rcvd data
        std::list<SITL_State::CANDriver*>::iterator can_mgr;
        for(can_mgr=_can_mgr.begin();can_mgr!=_can_mgr.end();can_mgr++) {
            if (((*can_mgr)->driver_.pending_frame_sent() || ! (*can_mgr)->driver_.isRxBufferEmpty()) &&  (*can_mgr)->_irq_handler_ctx) {
                pthread_mutex_lock(&(*can_mgr)->_irq_handler_mtx);
                pthread_cond_signal(&(*can_mgr)->_irq_handler_cond);
                pthread_mutex_unlock(&(*can_mgr)->_irq_handler_mtx);
            }
        }
    }

}

/*
  setup CAN pipes
 */
int SITL_State::can_sitl2ap_pipe(uint8_t idx)
{
    int fd[2];
    if (SITL_State::CANDriver::client_read_fd[idx] != -1) {
        return SITL_State::CANDriver::client_read_fd[idx];
    }
    pipe(fd);
    SITL_State::CANDriver::self_write_fd[idx]    = fd[1];
    SITL_State::CANDriver::client_read_fd[idx] = fd[0];
    fcntl(fd[0], F_SETFD, FD_CLOEXEC);
    fcntl(fd[1], F_SETFD, FD_CLOEXEC);
    HALSITL::UARTDriver::_set_nonblocking(SITL_State::CANDriver::self_write_fd[idx]);
    return SITL_State::CANDriver::client_read_fd[idx];
}

int SITL_State::can_ap2sitl_pipe(uint8_t idx)
{
    int fd[2];
    if (SITL_State::CANDriver::client_write_fd[idx] != -1) {
        return SITL_State::CANDriver::client_write_fd[idx];
    }
    pipe(fd);
    SITL_State::CANDriver::self_read_fd[idx]    = fd[0];
    SITL_State::CANDriver::client_write_fd[idx] = fd[1];
    fcntl(fd[0], F_SETFD, FD_CLOEXEC);
    fcntl(fd[1], F_SETFD, FD_CLOEXEC);
    HALSITL::UARTDriver::_set_nonblocking(fd[1]);
    return SITL_State::CANDriver::client_write_fd[idx];
}

// Initialise Emulated UAVCAN peripherals, to be controlled by respective drivers' sitl drivers
SITL_State::CANDriver* SITL_State::initNode(uint8_t driver_index, uint8_t node_id)
{
    can_sitl2ap_pipe(driver_index);
    can_ap2sitl_pipe(driver_index);

    SITL_State::CANDriver* can_mgr = new SITL_State::CANDriver(this);
    if (can_mgr != nullptr) {
        _num_nodes++;
        _can_mgr.push_back(can_mgr);
    } else {
        printf("SITLCAN: CAN%d driver not allocated\n\r", driver_index);
        return nullptr;
    }
    
    can_mgr->begin(1000000, driver_index);
    if (!can_mgr->is_initialized()) {
        printf("SITLCAN:  CAN%d driver not initialized\n\r", driver_index);
        return nullptr;
    }

    uavcan::ICanDriver* driver = can_mgr;
    uavcan::PoolAllocator<UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_BLOCK_SIZE, SITL_State::RaiiSynchronizer>* node_allocator = new uavcan::PoolAllocator<UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_BLOCK_SIZE, SITL_State::RaiiSynchronizer>;
    uavcan::Node<0>* node = new uavcan::Node<0>(*driver, SystemClock::instance(), *node_allocator);
    _node_allocator.push_back(node_allocator);

    if (node == nullptr) {
        printf("SITLCAN: couldn't allocate node\n\r");
        return nullptr;
    }

    if (node->isStarted()) {
        printf("SITLCAN: node was already started?\n\r");
        return nullptr;
    }
    int _uavcan_node = node_id;
    uavcan::NodeID self_node_id(_uavcan_node);
    node->setNodeID(self_node_id);

    char ndname[20];
    snprintf(ndname, sizeof(ndname), "org.sitlcan:%u", _num_nodes);

    uavcan::NodeStatusProvider::NodeName name(ndname);
    node->setName(name);

    uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
    sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
    node->setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion

    hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
    hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;

    const uint8_t uid_buf_len = hw_version.unique_id.capacity();
    uint8_t uid_len = uid_buf_len;
    uint8_t unique_id[uid_buf_len];


    if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
        //This is because we are maintaining a common Server Record for all UAVCAN Instances.
        //In case the node IDs are different, and unique id same, it will create
        //conflict in the Server Record.
        unique_id[uid_len - 1] += _uavcan_node;
        uavcan::copy(unique_id, unique_id + uid_len, hw_version.unique_id.begin());
    }
    node->setHardwareVersion(hw_version);

    int start_res = node->start();
    if (start_res < 0) {
        printf("SITLCAN: node start problem, error %d\n\r", start_res);
        return nullptr;
    }
    /*
     * Informing other nodes that we're ready to work.
     * Default mode is INITIALIZING.
     */
    node->setModeOperational();

    // Spin node for device discovery
    {
        WITH_SEMAPHORE(CANDriver::get_sem());
        can_mgr->_node = node;
        if (!_uc_periph_thread_created) {
            if (!hal.scheduler->thread_create(FUNCTOR_BIND(this, &SITL_State::uavcan_loop, void), "uc_periph", 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
                node->setModeOfflineAndPublish();
                printf("CANSITL: couldn't create thread\n\r");
                return nullptr;
            }
            _uc_periph_thread_created = true;
        }
        _node.push_back(node);
    }
    return can_mgr;
}

// We run single UAVCAN thread for all peripherals
void SITL_State::uavcan_loop(void) {
    while (true) {
        {
            WITH_SEMAPHORE(CANDriver::get_sem());
            std::list<uavcan::Node<0>*>::iterator node;
            for(node=_node.begin();node!=_node.end();node++) {
                if (*node == nullptr) {
                    continue;
                }
                (*node)->spinOnce();
            }
        }
        
        hal.scheduler->delay_microseconds(1000);
    }
}
