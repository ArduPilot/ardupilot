/// Copyright (c) 2018 Code Ex Machina, LLC. All rights reserved.
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program.If not, see <https://www.gnu.org/licenses/>.

#ifndef BlockingCollection_h
#define BlockingCollection_h

#include <unordered_set>
#include <condition_variable>
#include <type_traits>
#include <thread>
#include <chrono>
#include <deque>
#include <algorithm>

namespace code_machina {

    template <typename ConditionVarType,  typename LockType>
    struct ConditionVarTraits;

    template <>
    struct ConditionVarTraits<std::condition_variable, std::mutex> {
        static void initialize(std::condition_variable& cond_var) {
        }

        static void signal(std::condition_variable& cond_var) {
            cond_var.notify_one();
        }

        static void broadcast(std::condition_variable& cond_var) {
            cond_var.notify_all();
        }

        static void wait(std::condition_variable& cond_var,
        std::unique_lock<std::mutex>& lock) {
            cond_var.wait(lock);
        }

        template<class Rep, class Period> static bool wait_for(
            std::condition_variable& cond_var,
            std::unique_lock<std::mutex>& lock,
            const std::chrono::duration<Rep, Period>& rel_time) {
            return std::cv_status::timeout == cond_var.wait_for(lock, rel_time);
        }
    };

    /// @class ConditionVariable
    /// The ConditionVariable class wraps a operating system ConditionVariable.
    ///
    /// In addition, it implements support for attaching and detaching workers
    /// to the condition variable.
    /// @tparam ThreadContainerType The type of thread Container.
    /// @tparam SignalStrategyType The type of signal policy.
    /// @see NotEmptySignalStrategy
    /// @see NotFullSignalStrategy
    template<typename ThreadContainerType, typename SignalStrategyType,
    typename ConditionVarType, typename LockType>
    class ConditionVariable {
    public:
        /// Initializes a new instance of the ConditionVariable class without an
        /// upper-bound.
        ConditionVariable()
        : total_workers_(0), active_workers_(0), bounded_capacity_(SIZE_MAX),
        item_count_(0) {
        ConditionVarTraits<ConditionVarType, LockType>::initialize(
            condition_var_);
        }

        ~ConditionVariable() {
        }

        // "ConditionVariable" objects cannot be copied or assigned
        ConditionVariable(const ConditionVariable&) = delete;
        ConditionVariable& operator=(const ConditionVariable&) = delete;

        /// Gets the number of workers attached to this condition variable.
        /// @return The number of workers attached to this condition variable.
        /// @see Attach
        size_t total() const {
            return total_workers_;
        }

        /// Gets the number of active workers for this condition variable.
        /// active workers are workers that are currently NOT waiting on this
        /// condition variable.
        /// @return The number of active workers.
        size_t active() const {
            return active_workers_;
        }

        /// Gets the bounded capacity of this condition variable instance.
        /// @return The bounded capacity of this condition variable.
        size_t bounded_capacity() const {
            return bounded_capacity_;
        }

        /// Sets the bounded capacity of this condition variable instance.
        void bounded_capacity(size_t capacity) {
            bounded_capacity_ = capacity;
        }

        /// Gets the number of items contained in this condition variable
        /// instance.
        /// @return The number of items
        size_t size() const {
            return item_count_;
        }

        /// Set the number of items contained in this condition variable
        /// instance.
        void size(size_t count) {
            item_count_ = count;
        }

        /// Registers the a worker with this condition variable.
        /// If the worker is already registered then this method has no effect.
        /// @see Detach
        void attach() {
            if (container_.add()) {
                increment_total();
                increment_active();
            }
        }

        /// Unregisters the worker from this condition variable.
        /// If the worker was not previously registered then this method has
        /// no effect.
        /// @see Attach
        void detach() {
            if (container_.remove()) {
                decrement_total();
                decrement_active();
            }

            if (total_workers_ > 0 && active_workers_ == 0) {
                increment_active();
                ConditionVarTraits<ConditionVarType, LockType>::signal(
                    condition_var_);
            }
        }

        /// Wakes up a worker waiting on this condition variable.
        void signal() {
            // if no workers attached always signal!
            if (total_workers_ == 0) {
                ConditionVarTraits<ConditionVarType, LockType>::signal(
                    condition_var_);
                return;
            }
            // issue a signal only when there are no active workers, or when
            // the count starts to grow beyond a threshold level
            if (signal_.should_signal(active_workers_, total_workers_,
            item_count_, bounded_capacity_)) {
                increment_active();
                ConditionVarTraits<ConditionVarType, LockType>::signal(
                    condition_var_);
            }
        }

        /// Wakes up all workers waiting on this condition variable.
        void broadcast() {
            if (total_workers_ != 0) {
                // set active only if workers attached
                active(total_workers_);
            }
            ConditionVarTraits<ConditionVarType, LockType>::broadcast(
                condition_var_);
        }

        /// Waits indefinitely for this condition variable to become signaled.
        /// @param lock An object of type std::unique_lock which must be locked
        /// by the current thread.
        void wait(std::unique_lock<LockType>& lock) {
            decrement_active();
            ConditionVarTraits<ConditionVarType, LockType>::wait(
                condition_var_, lock);
        }

        /// Waits up to specified duration for this condition variable to become
        /// signaled.
        /// @param lock An object of type std::unique_lock which must be locked
        /// by the current thread.
        /// @param rel_time An object of type std::chrono::duration representing
        /// the maximum time to spend waiting.
        template<class Rep, class Period> bool wait_for(
            std::unique_lock<LockType>& lock,
            const std::chrono::duration<Rep, Period>& rel_time) {
            decrement_active();

            bool timed_out =
            ConditionVarTraits<ConditionVarType, LockType>::wait_for(
                condition_var_, lock, rel_time);

            if (timed_out) {
                increment_active();
            }

            return timed_out;
        }

    private:
        /// Sets the number of active workers for this condition variable.
        /// @param active The number of active workers.
        void active(size_t active) {
            active_workers_ = active > total_workers_ ? total_workers_ : active;
        }

        /// Increments the total worker count for this condition variable by 1.
        void increment_total() {
            total_workers_ += 1;
        }

        /// Decrements the total worker count for this condition variable by 1.
        void decrement_total() {
            total_workers_ = total_workers_ > 0 ? total_workers_ - 1 : 0;
        }

        /// Increments the active worker count for this condition variable by 1.
        void increment_active() {
            if (++active_workers_ > total_workers_)
                active_workers_ = total_workers_;
        }

        /// Decrements the active worker count for this condition variable by 1.
        void decrement_active() {
            active_workers_ = active_workers_ > 0 ? active_workers_ - 1 : 0;
        }

        size_t total_workers_;
        size_t active_workers_;
        size_t bounded_capacity_;
        size_t item_count_;

        ConditionVarType condition_var_;
        ThreadContainerType container_;
        SignalStrategyType signal_;
    };

    /// @class NotEmptySignalStrategy
    ///
    /// A strategy object for determining whether or not a "not empty" condition
    /// variable should issue a signal.
    ///
    /// This strategy will only return true if there are no active workers
    /// (i.e. all workers are waiting
    /// on empty BlockingCollection). Or when the BlockingCollection's element
    /// count starts to grow beyond a
    /// threshold level.
    ///
    /// This approach minimizes condition variable sleeps, wakes and lock
    /// contention. Which in turn,
    /// improves performance and makes it more predictable.
    /// @tparam ItemsPerThread The number of items to allow per thread.
    /// @see ConditionVariable
    /// @see NotFullSignalStrategy
    template<size_t ItemsPerThread = 16> struct NotEmptySignalStrategy {
        bool should_signal(size_t active_workers, size_t total_workers,
        size_t item_count, size_t /*capacity*/) const {
            return active_workers == 0 || (active_workers < total_workers &&
            item_count / active_workers > ItemsPerThread);
        }
    };

    /// @class NotFullSignalStrategy
    ///
    /// A strategy object for determining whether or not a "not full" condition
    ///  variable should issue a signal.
    ///
    /// This strategy will only return true if there are no active workers
    /// (i.e. all workers are
    /// waiting on a full BlockingCollection). Or when the BlockingCollection's
    /// available capacity
    /// starts to grow beyond a threshold level.
    ///
    /// This approach minimizes condition variable sleeps, wakes and lock
    /// contention. Which in turn,
    /// improves performance and makes it more predictable.
    /// @tparam ItemsPerThread The number of items to allow per thread.
    /// @see ConditionVariable
    /// @see NotEmptySignalStrategy
    template<size_t ItemsPerThread = 16> struct NotFullSignalStrategy {
        bool should_signal(size_t active_workers, size_t total_workers,
        size_t item_count, size_t capacity) const {
            return (active_workers == 0 || (active_workers < total_workers &&
            (capacity - item_count) / active_workers > ItemsPerThread));
        }
    };

    /// @class ConditionVariableGenerator
    ///
    /// Generates the "not full" and "not empty" condition variables for
    /// the specified ThreadContainerType.
    ///
    /// @tparam ThreadContainerType The thread Container policy to use when
    /// generating the condition variables.
    template<typename ThreadContainerType, typename NotFullSignalStrategy,
    typename NotEmptySignalStrategy, typename ConditionVarType,
    typename LockType> struct ConditionVariableGenerator {
        using NotFullType = ConditionVariable<ThreadContainerType,
        NotFullSignalStrategy, ConditionVarType, LockType>;
        using NotEmptyType = ConditionVariable<ThreadContainerType,
        NotEmptySignalStrategy, ConditionVarType, LockType>;

        using lock_type = LockType;
    };

    template <typename T>
    struct ThreadContainerTraits;

    template <>
    struct ThreadContainerTraits<std::thread::id> {
        static std::thread::id get_thread_id() {
            return std::this_thread::get_id();
        }
    };

    /// @class ThreadContainer
    /// This class adds and removes the specified thread type from the
    /// Container.
    /// @tparam T The thread type.
    template<typename T> class ThreadContainer {
    public:
        ThreadContainer() {
        }

        /// Adds the calling thread to the Container.
        /// @returns True if the calling thread was added to Container.
        /// Otherwise false.
        bool add() {
            T id = ThreadContainerTraits<T>::get_thread_id();

            typename std::unordered_set<T>::iterator itr = thread_id_.find(id);

            if (itr != thread_id_.end()) {
                return false;
            }

            thread_id_.insert(id);
            return true;
        }

        /// Removes the calling thread from the Container.
        /// @returns True if the calling thread was removed from Container.
        /// Otherwise false.
        bool remove() {
            if (thread_id_.erase(ThreadContainerTraits<T>::get_thread_id())
            > 0) {
                return true;
            }
            return false;
        }

    private:
        std::unordered_set<T> thread_id_;
    };

    namespace detail {
        struct QueueType {};
        struct StackType {};

        template< typename T >
        struct is_queue : std::false_type { };

        template<>
        struct is_queue<QueueType> : std::true_type {};

        /// @class Container
        ///
        /// Represents a first in-first out (FIFO) or a last in-first out
        /// (LIFO) collection depending on
        /// the ContainerType template parameter value.
        ///
        /// Implements the implicitly defined IProducerConsumerCollection<T>
        /// policy.
        /// @tparam T The type of items in the Container.
        /// @tparam ContainerType The type of Container (i.e. Queue or Stack).
        template<typename T, typename ContainerType>
        class Container {
        public:
            using container_type = std::deque<T>;
            using value_type = typename container_type::value_type;
            using size_type = typename container_type::size_type;

            /// Initializes a new instance of the Container<T> class.
            Container()
            : bounded_capacity_(SIZE_MAX)  {
            }

            /// Sets the max number of elements this container can hold.
            /// @param bounded_capacity The max number of elements this
            /// container can hold.
            void bounded_capacity(size_t bounded_capacity) {
                bounded_capacity_ = bounded_capacity;
            }

            /// Gets the max number of elements this container can hold.
            /// @returns The max number of elements this container can hold.
            size_t bounded_capacity() {
                return bounded_capacity_;
            }

            /// Gets the number of elements contained in the collection.
            /// @returns The number of elements contained in the collection.
            size_type size() {
                return container_.size();
            }

            /// Attempts to add an element to the collection.
            /// @param item The element to add to the collection.
            /// @returns True if the element was added successfully; otherwise,
            /// false.
            bool try_add(const value_type& item) {
                if (container_.size() == bounded_capacity_)
                    return false;
                container_.push_back(item);
                return true;
            }

            /// Attempts to add an element to the collection.
            /// @param item The element to add to the collection.
            /// @returns True if the element was added successfully; otherwise,
            /// false.
            bool try_add(value_type&& item) {
                if (container_.size() == bounded_capacity_)
                    return false;
                container_.push_back(std::forward<value_type>(item));
                return true;
            }

            /// Attempts to remove and return an element from the collection.
            /// @param [out] item When this method returns, if the element was
            /// removed and returned successfully, item
            /// contains the removed element. If no element was available to be
            /// removed, the value is unspecified.
            /// @returns True if an element was removed and returned
            /// successfully; otherwise, false.
            bool try_take(value_type& item) {
                if (container_.empty())
                    return false;
                return try_take_i(item, is_queue<ContainerType>());
            }

            /// Attempts to add an element to the collection.
            /// This new element is constructed in place using args as the
            /// arguments for its construction.
            /// @param args Arguments forwarded to construct the new element.
            template <typename... Args> bool try_emplace(Args&&... args) {
                if (container_.size() == bounded_capacity_)
                    return false;
                return try_emplace_i<Args...>(std::forward<Args>(args)...,
                is_queue<ContainerType>());
            }

        private:
            size_t bounded_capacity_;
            container_type container_;

            bool try_take_i(value_type& item, std::false_type) {
                item = container_.back();
                container_.pop_back();
                return true;
            }

            bool try_take_i(value_type& item, std::true_type) {
                item = container_.front();
                container_.pop_front();
                return true;
            }

            template <typename... Args> bool try_emplace_i(Args&&... args,
            std::false_type) {
                container_.emplace_front(std::forward<Args>(args)...);
                return true;
            }

            template <typename... Args> bool try_emplace_i(Args&&... args,
            std::true_type) {
                container_.emplace_back(std::forward<Args>(args)...);
                return true;
            }
        };
    } // namespace detail

    template<typename T>
    using QueueContainer = detail::Container<T, detail::QueueType>;

    template<typename T>
    using StackContainer = detail::Container<T, detail::StackType>;

    using StdConditionVariableGenerator = ConditionVariableGenerator<
    ThreadContainer<std::thread::id>, NotFullSignalStrategy<16>,
    NotEmptySignalStrategy<16>, std::condition_variable, std::mutex>;

    /// @enum BlockingCollectionState
    /// The BlockCollection states.
    enum class BlockingCollectionState {
        // BlockingCollection is active and processing normally.
        Activated = 1,
        // BlockingCollection is deactivated; no add or take operations allowed.
        Deactivated = 2,
        // BlockingCollection was pulsed; add and take may proceed normally.
        Pulsed = 3
    };

    /// @enum BlockingCollectionStatus
    /// The BlockCollection status codes.
    /// These are the status codes returned by all of BlockingCollection's Add
    /// and Take operations.
    enum class BlockingCollectionStatus {
        /// Operation succeeded
        Ok = 0,
        /// Operation failed due to CompleteAdding() having been invoked
        AddingCompleted = -1,
        /// Operation failed due to time out
        TimedOut = -2,
        /// Operation failed due to BlockingCollection not being activated
        NotActivated = -3,
        /// Operation failed due to BlockingCollection being completed
        Completed = -4,
        /// Operation failed due to invalid iterators
        InvalidIterators = -5,
        /// Operation failed due to concurrent Add and CompleteAdding
        CompleteAddingConcurrent = -6,
        /// Operation failed due to BlockingCollection Container error
        InternalError = -8
    };

    template <typename T, typename ContainerType = QueueContainer<T>,
    typename ConditionVariableGenerator = StdConditionVariableGenerator>
    class BlockingCollection {
    public:
        using LockType = typename ConditionVariableGenerator::lock_type;

        /// Initializes a new instance of the BlockingCollection<T> class
        /// without an upper-bound.
        BlockingCollection()
        : BlockingCollection(SIZE_MAX) {
        }

        /// Initializes a new instance of the BlockingCollection<T> class
        /// with the specified upper-bound.
        /// @param capacity The bounded size of the collection.
        explicit BlockingCollection(size_t capacity)
        : state_(BlockingCollectionState::Activated),
        bounded_capacity_(capacity),
        is_adding_completed_(false) {
            not_empty_condition_var_.bounded_capacity(capacity);
            not_full_condition_var_.bounded_capacity(capacity);
            container_.bounded_capacity(capacity);
        }

        // "BlockingCollection" objects cannot be copied or assigned
        BlockingCollection(const BlockingCollection&) = delete;
        BlockingCollection& operator=(const BlockingCollection&) = delete;

        ~BlockingCollection() {
        }

        /// Gets the bounded capacity of this BlockingCollection<T> instance.
        /// @return The bounded capacity of the collection.
        size_t bounded_capacity() {
            std::lock_guard<LockType> guard(lock_);
            return bounded_capacity_;
        }

        /// Gets the current state of this BlockingCollection<T> instance.
        /// @return The current state of the collection.
        /// @see BlockingCollectionState
        BlockingCollectionState state() {
            std::lock_guard<LockType> guard(lock_);
            return state_;
        }

        /// Gets whether this BlockingCollection<T> instance is full.
        /// @return True if the collection is full; otherwise false.
        bool is_full() {
            std::lock_guard<LockType> guard(lock_);
            return is_full_i();
        }

        /// Gets whether this BlockingCollection<T> instance is empty.
        /// @return True if the collection is empty; otherwise false.
        bool is_empty() {
            std::lock_guard<LockType> guard(lock_);
            return is_empty_i();
        }

        /// Gets the number of items contained in the BlockingCollection<T>
        /// instance.
        /// If any method in BlockingCollection is executing while the size
        /// property is being accessd, the return value
        /// is approximate. size may reflect a number that is either greater
        /// than or less than the actual number of
        /// items in the BlockingCollection.
        /// @return The number of item in the collection.
        size_t size() {
            std::lock_guard<LockType> guard(lock_);
            return container_.size();
        }

        /// Gets whether this BlockingCollection<T> instance has been
        /// deactivated.
        /// @return True is this collection has been deactivated.
        /// Otherwise false.
        bool is_deactivated() {
            std::lock_guard<LockType> guard(lock_);
            return state_ == BlockingCollectionState::Deactivated;
        }

        /// Gets whether this BlockingCollection<T> instance has been marked
        /// as complete for adding and is empty.
        /// @return True if this collection has been marked as complete for
        /// adding and is empty. Otherwise false.
        bool is_completed() {
            std::lock_guard<LockType> guard(lock_);
            return is_completed_i();
        }

        /// Gets whether this BlockingCollection<T> instance has been marked
        /// as complete for adding.
        /// @return True if this collection has been marked as complete for
        /// adding. Otherwise false.
        bool is_adding_completed() {
            std::lock_guard<LockType> guard(lock_);
            return is_adding_completed_i();
        }

        /// Pulses this BlockingCollection<T> instance to wake up any waiting
        /// threads.
        /// Changes the collection's state to Pulsed. Future Add and Take
        /// operations proceed
        /// as in the Activated state.
        /// @return The BlockingCollection's state before this call.
        /// @see BlockingCollectionState
        BlockingCollectionState pulse() {
            std::lock_guard<LockType> guard(lock_);
            return deactivate_i(true);
        }

        /// Deactivate this BlockingCollection<T> instance and wakeup all
        /// threads waiting
        /// on the collection so they can continue. No items are removed from
        /// the collection,
        /// however. Any other operations called until the collection is
        /// activated again will immediately return
        /// BlockingCollectionStatus::NotActivated.
        /// @return The BlockingCollection's state before this call.
        /// @see BlockingCollectionState
        /// @see BlockingCollectionStatus
        BlockingCollectionState deactivate() {
            std::lock_guard<LockType> guard(lock_);
            return deactivate_i(false);
        }

        /// Reactivate this BlockingCollection<T> instance so that threads
        /// can Add and Take
        /// items again.
        /// @return The BlockingCollection's state before this call.
        /// @see BlockingCollectionState
        BlockingCollectionState activate() {
            std::lock_guard<LockType> guard(lock_);
            return activate_i();
        }

        /// Releases all items from this BlockingCollection<T> instance
        /// but does not mark it deactivated.
        /// @return The number of items flushed.
        size_t flush() {
            std::lock_guard<LockType> guard(lock_);

            auto itemsFlushed = container_.size();

            T item;

            while (container_.size() > 0) {
                container_.try_take(item);
            }

            not_empty_condition_var_.size(0);
            not_full_condition_var_.size(0);

            return itemsFlushed;
        }

        /// Marks the BlockingCollection<T> instances as not accepting any more
        /// additions.
        /// After a collection has been marked as complete for adding, adding
        /// to the collection
        /// is not permitted and attempts to remove from the collection will
        /// not wait when the collection is empty.
        void complete_adding() {
            std::lock_guard<LockType> guard(lock_);

            if (is_adding_completed_)
                return;

            is_adding_completed_ = true;

            not_empty_condition_var_.broadcast();
            not_full_condition_var_.broadcast();
        }

        /// Gets the number of consumer threads that are actively taking items
        /// from this BlockingCollection<T> instance.
        /// @return The number of active consumer threads.
        /// @see AttachConsumer
        size_t active_consumers() {
            std::lock_guard<LockType> guard(lock_);
            return not_empty_condition_var_.active();
        }

        /// Gets the number of producer threads that are actively adding items
        /// to this BlockingCollection<T> instance.
        /// @return The number of active producer threads.
        /// @see AttachProducer
        size_t active_producers() {
            std::lock_guard<LockType> guard(lock_);
            return not_full_condition_var_.active();
        }

        /// Gets the total number of consumer threads that can take items
        /// from this BlockingCollection<T> instance.
        /// @return The total number of consumer threads.
        /// @see AttachConsumer
        size_t total_consumers() {
            std::lock_guard<LockType> guard(lock_);
            return not_empty_condition_var_.total();
        }

        /// Gets the total number of producer threads that can add items
        /// to this BlockingCollection<T> instance.
        /// @return The total number of producer threads.
        /// @see AttachProducer
        size_t total_producers() {
            std::lock_guard<LockType> guard(lock_);
            return not_full_condition_var_.total();
        }

        /// Registers a consumer thread with this BlockingCollection<T>
        /// instance.
        /// @see TotalConsumers
        void attach_consumer() {
            std::lock_guard<LockType> guard(lock_);
            not_empty_condition_var_.attach();
        }

        /// Unregisters a consumer thread with this BlockingCollection<T>
        /// instance.
        /// @see TotalConsumers
        void detach_consumer() {
            std::lock_guard<LockType> guard(lock_);
            not_empty_condition_var_.detach();
        }

        /// Registers a producer thread with this BlockingCollection<T>
        /// instance.
        /// @see TotalProducers
        void attach_producer() {
            std::lock_guard<LockType> guard(lock_);
            not_full_condition_var_.attach();
        }

        /// Unregisters a producer thread with this BlockingCollection<T>
        /// instance.
        /// @see TotalProducers
        void detach_producer() {
            std::lock_guard<LockType> guard(lock_);
            not_full_condition_var_.detach();
        }

        /// Adds the given element value to the BlockingCollection<T>.
        /// The new element is initialized as a copy of value.
        /// If a bounded capacity was specified when this instance of
        /// BlockingCollection<T> was initialized,
        /// a call to Add may block until space is available to store the
        /// provided item.
        /// @param value the value of the element to add
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        BlockingCollectionStatus add(const T& value) {
            return try_emplace_timed(std::chrono::milliseconds(-1), value);
        }

        /// Adds the given element value to the BlockingCollection<T>.
        /// Value is moved into the new element.
        /// If a bounded capacity was specified when this instance of
        /// BlockingCollection<T> was initialized,
        /// a call to Add may block until space is available to store the
        /// provided item.
        /// @param value the value of the element to add
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        BlockingCollectionStatus add(T&& value) {
            return try_emplace_timed(std::chrono::milliseconds(-1),
            std::forward<T>(value));
        }

        /// Tries to add the given element value to the BlockingCollection<T>.
        /// The new element is initialized as a copy of value.
        /// If the collection is a bounded collection, and is full, this method
        /// immediately returns without adding the item.
        /// @param value the value of the element to try to add
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        BlockingCollectionStatus try_add(const T& value) {
            return try_emplace_timed(std::chrono::milliseconds::zero(), value);
        }

        /// Tries to add the given element value to the BlockingCollection<T>.
        /// Value is moved into the new element.
        /// If the collection is a bounded collection, and is full, this
        /// method immediately returns without adding the item.
        /// @param value the value of the element to try to add
        BlockingCollectionStatus try_add(T&& value) {
            return try_emplace_timed(std::chrono::milliseconds::zero(),
            std::forward<T>(value));
        }

        /// Tries to add the given element value to the BlockingCollection<T>
        /// within the specified time period.
        /// Value is moved into the new element.
        /// @param value the value of the element to try to add
        /// @param rel_time An object of type std::chrono::duration
        /// representing the maximum time to spend waiting.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/duration
        template<typename U, class Rep, class Period>
        BlockingCollectionStatus try_add_timed(U&& value,
        const std::chrono::duration<Rep, Period>& rel_time) {
            return try_emplace_timed(rel_time, std::forward<U>(value));
        }

        /// Adds new element to the BlockingCollection<T>.
        /// The arguments args... are forwarded to the constructor as
        /// std::forward<Args>(args)....If a bounded capacity was specified
        /// when this instance of BlockingCollection<T> was initialized,
        /// a call to Emplace may block until space is available to store the
        /// provided item.
        /// @param args arguments to forward to the constructor of the element
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template<typename... Args>
        BlockingCollectionStatus emplace(Args&&... args) {
            return try_emplace_timed(std::chrono::milliseconds(-1),
            std::forward<Args>(args)...);
        }

        /// Tries to add new element to the BlockingCollection<T>.
        /// The arguments args... are forwarded to the constructor as
        /// std::forward<Args>(args)....
        /// If the collection is a bounded collection, and is full, this method
        /// immediately
        /// returns without adding the item.
        /// @param args arguments to forward to the constructor of the element
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template<typename... Args>
        BlockingCollectionStatus try_emplace(Args&&... args) {
            return try_emplace_timed(std::chrono::milliseconds::zero(),
            std::forward<Args>(args)...);
        }

        /// Tries to add the given element value to the BlockingCollection<T>
        /// within the specified time period.
        /// The arguments args... are forwarded to the constructor as
        /// std::forward<Args>(args)....
        /// If the collection is a bounded collection, and is full, this
        /// method immediately returns without adding the item.
        /// @param args arguments to forward to the constructor of the element
        /// @param rel_time An object of type std::chrono::duration
        /// representing the maximum time to spend waiting.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/duration
        template<class Rep, class Period, typename... Args>
        BlockingCollectionStatus try_emplace_timed(
            const std::chrono::duration<Rep, Period>& rel_time,
            Args&&... args) {
            {
                std::unique_lock<LockType> guard(lock_);

                auto status = wait_not_full_condition(guard, rel_time);

                if (BlockingCollectionStatus::Ok != status)
                    return status;

                if (!container_.try_emplace(std::forward<Args>(args)...))
                    return BlockingCollectionStatus::InternalError;

                signal(container_.size(), false);
            }
            return BlockingCollectionStatus::Ok;
        }

        /// Removes an item from the BlockingCollection<T>.
        /// A call to Take may block until an item is available to be removed.
        /// @param[out] item The item removed from the collection.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        BlockingCollectionStatus take(T& item) {
            return try_take(item, std::chrono::milliseconds(-1));
        }

        /// Tries to remove an item from the BlockingCollection<T>.
        /// If the collection is empty, this method immediately returns without
        /// taking an item.
        /// @param[out] item The item removed from the collection.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        BlockingCollectionStatus try_take(T& item) {
            return try_take(item, std::chrono::milliseconds::zero());
        }

        /// Tries to remove an item from the BlockingCollection<T> in the
        /// specified time period.
        /// @param[out] item The item removed from the collection.
        /// @param rel_time An object of type std::chrono::duration
        /// representing the maximum time to spend waiting.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/duration
        template<class Rep, class Period> BlockingCollectionStatus
        try_take(T& item, const std::chrono::duration<Rep, Period>& rel_time) {
            {
                std::unique_lock<LockType> guard(lock_);

                auto status = wait_not_empty_condition(guard, rel_time);

                if (BlockingCollectionStatus::Ok != status)
                    return status;

                if (!container_.try_take(item))
                    return BlockingCollectionStatus::InternalError;

                signal(container_.size(), true);
            }
            return BlockingCollectionStatus::Ok;
        }

        /// Adds the items from range [first, last] to the
        /// BlockingCollection<T>.
        /// If a bounded capacity was specified when this instance of
        /// BlockingCollection<T> was initialized,
        /// a call to Add may block until space is available to store the
        /// provided items.
        /// Use std::make_move_iterator if the elements should be moved
        /// instead of copied.
        /// @param first The start range of elements to insert.
        /// @param last The end range of elements to insert.
        /// @param [out] added The actual number of elements added.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template <typename Iterator> BlockingCollectionStatus
        add_bulk(Iterator first, Iterator last, size_t& added) {
            return try_add_bulk(first, last, added,
            std::chrono::milliseconds(-1));
        }

        /// Tries to add the items from range [first, last] to the
        /// BlockingCollection<T>.
        /// If the collection is a bounded collection, and is full, this method
        /// immediately returns without adding the items.
        /// Use std::make_move_iterator if the elements should be moved
        /// instead of copied.
        /// @param first The start range of elements to insert.
        /// @param last The end range of elements to insert.
        /// @param [out] added The actual number of elements added.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template <typename Iterator> BlockingCollectionStatus
        try_add_bulk(Iterator first, Iterator last, size_t& added) {
            return try_add_bulk(first, last, added,
            std::chrono::milliseconds::zero());
        }

        /// Tries to add the specified items from the range [first, last] to
        /// the BlockingCollection<T> within
        /// the specified time period.
        /// Use std::make_move_iterator if the elements should be moved
        /// instead of copied.
        /// @param first The start range of elements to insert.
        /// @param last The end range of elements to insert.
        /// @param [out] added The actual number of elements added.
        /// @param rel_time An object of type std::chrono::duration representing
        ///  the maximum time to spend waiting.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/durations
        template<typename Iterator, class Rep, class Period>
        BlockingCollectionStatus try_add_bulk(Iterator first, Iterator last,
        size_t& added, const std::chrono::duration<Rep, Period>& rel_time) {
            {
                added = 0;

                std::unique_lock<LockType> guard(lock_);

                auto status = wait_not_full_condition(guard, rel_time);

                if (BlockingCollectionStatus::Ok != status)
                    return status;

                if (first == last)
                    return BlockingCollectionStatus::InvalidIterators;

                for (; first != last; ++first) {
                    if (!container_.try_add((*first)))
                        break;
                    ++added;
                }

                signal(container_.size(), false);
            }
            return BlockingCollectionStatus::Ok;
        }

        /// Takes up to count elements from the BlockingCollection<T>.
        /// A call to take_bulk may block until an element is available to be
        /// removed.
        /// Use std::make_move_iterator if the elements should be moved instead
        /// of copied.
        /// @param[out] first Contains first item taken.
        /// @param count The number of elements to take from collection.
        /// @param[out] taken The actual number of elements taken.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template <typename Iterator> BlockingCollectionStatus
        take_bulk(Iterator first, size_t count, size_t& taken) {
            return try_take_bulk(first, count, taken,
            std::chrono::milliseconds(-1));
        }

        /// Takes up to count elements from the BlockingCollection<T>.
        /// If the collection is empty, this method immediately returns without
        /// taking any items.
        /// Use std::make_move_iterator if the elements should be moved instead
        /// of copied.
        /// @param[out] first Contains first item taken.
        /// @param count The number of elements to take from collection.
        /// @param[out] taken The actual number of elements taken.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template <typename Iterator> BlockingCollectionStatus
        try_take_bulk(Iterator first, size_t count, size_t& taken) {
            return try_take_bulk(first, count, taken,
            std::chrono::milliseconds::zero());
        }

        /// Tries to take up to count elements from the BlockingCollection<T>
        /// within the specified time period.
        /// If the collection is empty, this method immediately returns without
        /// taking any items.
        /// Use std::make_move_iterator if the elements should be moved instead
        /// of copied.
        /// @param[out] first Contains first item taken.
        /// @param count The number of elements to take from collection.
        /// @param[out] taken The actual number of elements taken.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/durations
        template <typename Iterator, class Rep, class Period>
        BlockingCollectionStatus try_take_bulk(Iterator first, size_t count,
        size_t& taken, const std::chrono::duration<Rep, Period>& rel_time) {
            {
                taken = 0;

                if (count == 0)
                    return BlockingCollectionStatus::Ok;

                std::unique_lock<LockType> guard(lock_);

                auto status = wait_not_empty_condition(guard, rel_time);

                if (BlockingCollectionStatus::Ok != status)
                    return status;

                auto end = first + count;

                for (; first != end; ++first) {
                    if (!container_.try_take((*first)))
                        break;

                    if (++taken == count)
                        break;
                }

                signal(container_.size(), true);
            }
            return BlockingCollectionStatus::Ok;
        }

    private:
    class Iterator {
    public:
        Iterator(BlockingCollection<T, ContainerType,
        ConditionVariableGenerator> &collection)
        : collection_(collection), status_(BlockingCollectionStatus::Ok),
        wait_for_first_item(true) {
        }

        Iterator(BlockingCollection<T,
        ContainerType, ConditionVariableGenerator> &collection,
        BlockingCollectionStatus status)
        : collection_(collection), status_(status), wait_for_first_item(false) {
        }

        // "Iterator" objects cannot be copied or assigned
        Iterator(const BlockingCollection&) = delete;
        Iterator& operator=(const Iterator&) = delete;

        bool operator!=(const Iterator& it)  {
            if (wait_for_first_item) {
                wait_for_first_item = false;
                status_ = collection_.try_take(item_,
                // -1 forces TryTake to wait
                std::chrono::milliseconds(-1));
            }

            return !(status_ != BlockingCollectionStatus::Ok);
        }

        Iterator& operator++() {
            status_ = collection_.try_take(item_,
            std::chrono::milliseconds(-1));
            return *this;
        }

        T& operator*() {
            return item_;
        }

    private:
        BlockingCollection<T, ContainerType, ConditionVariableGenerator>
        &collection_;
        BlockingCollectionStatus status_;
        bool wait_for_first_item;
        T item_;
    };

    public:
        Iterator begin() { return { *this }; }
        Iterator end()   { return { *this }; }

    private:
        // the member functions below assume lock is held!

        /// The implementation for the Deactivate method.
        /// This method is not thread safe.
        /// @see Deactivate
        BlockingCollectionState deactivate_i(bool pulse) {
            auto previous_state = state_;

            if (previous_state != BlockingCollectionState::Deactivated) {
                if (pulse)
                    state_ = BlockingCollectionState::Pulsed;
                else
                    state_ = BlockingCollectionState::Deactivated;

                not_empty_condition_var_.broadcast();
                not_full_condition_var_.broadcast();
            }

            return previous_state;
        }

        /// The implementation for the Activate method.
        /// This method is not thread safe.
        /// @see Activate
        BlockingCollectionState activate_i() {
            auto previous_state = state_;

            state_ = BlockingCollectionState::Activated;

            return previous_state;
        }

        /// The implementation for the is_full method.
        /// This method is not thread safe.
        /// @see is_full
        bool is_full_i() {
            return bounded_capacity_ != SIZE_MAX &&
            container_.size() >= bounded_capacity_;
        }

        /// The implementation for the is_empty method.
        /// This method is not thread safe.
        /// @see is_empty
        bool is_empty_i() {
            return container_.size() == 0;
        }

        /// The implementation for the is_completed method.
        /// This method is not thread safe.
        /// @see is_completed
        bool is_completed_i() {
            return is_adding_completed_ && is_empty_i();
        }

        /// The implementation for the is_adding_completed method.
        /// This method is not thread safe.
        /// @see is_adding_completed
        bool is_adding_completed_i() {
            return is_adding_completed_;
        }

    protected:
        /// Wraps the condition variable signal methods.
        /// This method updates the size property on both
        /// condition variables before invoking the signal
        /// method on the specified condition variable.
        void signal(size_t itemCount, bool signal_not_full) {
            not_empty_condition_var_.size(itemCount);
            not_full_condition_var_.size(itemCount);

            if (signal_not_full) {
                // signal only if capacity is bounded
                if (bounded_capacity_ != SIZE_MAX) {
                    not_full_condition_var_.signal();
                }
            } else {
                not_empty_condition_var_.signal();
            }
        }

        /// The method waits on the "not full" condition variable whenever
        /// the collection becomes full.
        /// It atomically releases lock, blocks the current executing thread,
        /// and adds it to the
        /// list of threads waiting on the "not full" condition variable. The
        /// thread will be unblocked
        /// when notify_all() or notify_one() is executed, or when the relative
        /// timeout rel_time expires.
        /// It may also be unblocked spuriously. When unblocked, regardless of
        /// the reason, lock is reacquired
        /// and wait_not_full_condition() exits.
        /// @param lock An object of type std::unique_lock which must be locked
        /// by the current thread.
        /// @param rel_time An object of type std::chrono::duration representing
        /// the maximum time to spend waiting.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/duration
        template<class Rep, class Period> BlockingCollectionStatus
        wait_not_full_condition(std::unique_lock<LockType>& lock,
        const std::chrono::duration<Rep, Period>& rel_time) {
            if (state_ == BlockingCollectionState::Deactivated)
                return BlockingCollectionStatus::NotActivated;

            if (is_adding_completed_i())
                return BlockingCollectionStatus::AddingCompleted;

            auto status = BlockingCollectionStatus::Ok;

            while (is_full_i()) {
                if (rel_time == std::chrono::duration<Rep, Period>::zero()) {
                    status = BlockingCollectionStatus::TimedOut;
                    break;
                }

                if (is_adding_completed_i()) {
                    status = BlockingCollectionStatus::AddingCompleted;
                    break;
                }

                if (rel_time.count() < 0) {
                    not_full_condition_var_.wait(lock);
                } else {
                    if (not_full_condition_var_.wait_for(lock, rel_time)) {
                        status = BlockingCollectionStatus::TimedOut;
                        break;
                    }
                }

                // Add/TryAdd methods and CompleteAdding should not
                // be called concurrently - invalid operation

                if (is_adding_completed_i()) {
                    status = BlockingCollectionStatus::CompleteAddingConcurrent;
                    break;
                }

                if (state_ != BlockingCollectionState::Activated) {
                    status = BlockingCollectionStatus::NotActivated;
                    break;
                }
            }
            return status;
        }

        /// The method waits on the "not empty" condition variable whenever the
        /// collection becomes empty.
        /// It atomically releases lock, blocks the current executing thread,
        /// and adds it to the
        /// list of threads waiting on the "not empty" condition variable. The
        /// thread will be unblocked
        /// when notify_all() or notify_one() is executed, or when the relative
        /// timeout rel_time expires.
        /// It may also be unblocked spuriously. When unblocked, regardless of
        /// the reason, lock is reacquired
        /// and wait_not_empty_condition() exits.
        /// @param lock An object of type std::unique_lock which must be locked
        /// by the current thread.
        /// @param rel_time An object of type std::chrono::duration representing
        /// the maximum time to spend waiting.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/duration
        template<class Rep, class Period> BlockingCollectionStatus
        wait_not_empty_condition(std::unique_lock<LockType>& lock,
        const std::chrono::duration<Rep, Period>& rel_time) {
            if (state_ == BlockingCollectionState::Deactivated)
                return BlockingCollectionStatus::NotActivated;

            if (is_completed_i())
                return BlockingCollectionStatus::Completed;

            auto status = BlockingCollectionStatus::Ok;

            while (is_empty_i()) {
                if (rel_time == std::chrono::duration<Rep, Period>::zero()) {
                    status = BlockingCollectionStatus::TimedOut;
                    break;
                }

                if (is_adding_completed_i()) {
                    status = BlockingCollectionStatus::AddingCompleted;
                    break;
                }

                if (rel_time.count() < 0) {
                    not_empty_condition_var_.wait(lock);
                } else {
                    if (not_empty_condition_var_.wait_for(lock, rel_time)) {
                        status = BlockingCollectionStatus::TimedOut;
                        break;
                    }
                }

                if (state_ != BlockingCollectionState::Activated) {
                    status = BlockingCollectionStatus::NotActivated;
                    break;
                }
            }

            return status;
        }

        ContainerType& container() {
            return container_;
        }

        LockType& lock() {
            return lock_;
        }

    private:
        BlockingCollectionState state_;

        size_t bounded_capacity_;
        bool is_adding_completed_;

        typename ConditionVariableGenerator::NotEmptyType not_empty_condition_var_;
        typename ConditionVariableGenerator::NotFullType not_full_condition_var_;

        // Synchronizes access to the BlockCollection.
        LockType lock_;
        // The underlying Container (e.g. Queue, Stack).
        ContainerType container_;
    };

    /// @class PriorityContainer
    /// Represents a priority based collection. Items with the highest priority
    /// will be at the head of the collection.
    /// Implements the implicitly defined IProducerConsumerCollection<T> policy.
    /// @tparam T The type of items in the collection.
    /// @tparam ComparerType The type of comparer to use when comparing items.
    template<typename T, typename ComparerType>
    class PriorityContainer {
    public:
        using container_type = std::deque<T>;
        using size_type = typename container_type::size_type;
        using value_type = typename container_type::value_type;
        using value_comparer = ComparerType;

        /// Initializes a new instance of the PriorityContainer<T> class.
        PriorityContainer()
        : bounded_capacity_(SIZE_MAX) {
        }

        /// Sets the max number of elements this container can hold.
        /// @param bounded_capacity The max number of elements this
        /// container can hold.
        void bounded_capacity(size_t bounded_capacity) {
            bounded_capacity_ = bounded_capacity;
        }

        /// Gets the max number of elements this container can hold.
        /// @returns The max number of elements this container can hold.
        size_t bounded_capacity() {
            return bounded_capacity_;
        }

        /// Gets the number of elements contained in the collection.
        /// @returns The number of elements contained in the collection.
        size_type size() {
            return container_.size();
        }

        /// Attempts to add an object to the collection according to the item's
        /// priority.
        /// @param new_item The object to add to the collection.
        /// @returns True if the object was added successfully; otherwise,
        /// false.
        bool try_add(const value_type& new_item) {
            if (container_.size() == bounded_capacity_)
                return false;
            return try_emplace(new_item);
        }

        /// Attempts to add an object to the collection according to the item's
        /// priority.
        /// @param new_item The object to add to the collection.
        /// @returns True if the object was added successfully; otherwise,
        /// false.
        bool try_add(value_type&& new_item) {
            if (container_.size() == bounded_capacity_)
                return false;
            return try_emplace(std::forward<T>(new_item));
        }

        /// Attempts to add an element to the collection according to the
        /// element's priority.
        /// This new element is constructed in place using args as the
        /// arguments for its construction.
        /// @param args Arguments forwarded to construct the new element.
        template <typename... Args> bool try_emplace(Args&&... args) {
            if (container_.size() == bounded_capacity_)
                return false;
            if (container_.empty()) {
                container_.emplace_front(std::forward<Args>(args)...);
            } else {
                T new_item(args...);

                // search from back to front (i.e. from the lowest priority to
                // the highest priority) for
                // item with a priority greater than or equal to new_item's
                // priority

                typename container_type::reverse_iterator itr = std::find_if(
                    container_.rbegin(), container_.rend(),
                    [&new_item, this](value_type &item) {
                    return this->comparer_(item, new_item) >= 0;
                });

                if (itr == container_.rend()) {
                    // if at end then new_item's priority is now the highest
                    container_.emplace_front(std::move(new_item));
                } else if (itr == container_.rbegin()) {
                    // if at start then new_item's priority is now the lowest
                    container_.emplace_back(std::move(new_item));
                } else {
                    // insert the new item behind the item of greater or
                    // equal priority. This ensures that FIFO order is
                    // maintained when items of the same priority are
                    // inserted consecutively.
                    container_.emplace(itr.base(), std::move(new_item));
                }
            }
            return true;
        }

        /// Attempts to remove and return the highest priority object from the
        /// collection.
        /// @param [out] item When this method returns, if the object was
        /// removed and returned successfully, item contains
        /// the removed object. If no object was available to be removed, the
        /// value is unspecified.
        /// @returns True if an object was removed and returned successfully;
        /// otherwise, false.
        bool try_take(value_type& item) {
            if (container_.empty())
                return false;
            item = container_.front();
            container_.pop_front();
            return true;
        }

        /// Attempts to remove and return the lowest priority object from the
        /// collection.
        /// @param [out] item When this method returns, if the object was
        /// removed and returned successfully, item contains
        /// the removed object. If no object was available to be removed, the
        /// value is unspecified.
        /// @returns True if an object was removed and returned successfully;
        /// otherwise, false.
        bool try_take_prio(value_type& item) {
            if (container_.empty())
                return false;

            bool init_current_priority = true;
            value_type* current_priority;

            typename container_type::reverse_iterator itr = std::find_if_not(
                container_.rbegin(), container_.rend(),
                [&current_priority, &init_current_priority, this](value_type &item) -> bool {
                    // Find the first version of the earliest item (i.e.,
                    // preserve FIFO order for items at the same priority).

                    if (init_current_priority) {
                        current_priority = &item;
                        init_current_priority = false;
                        return true;
                    }
                    bool continue_search = this->comparer_(item, *current_priority) <= 0;
                    if (continue_search) {
                        current_priority = &item;
                    }
                    return continue_search;
                });

            if (itr == container_.rend()) {
                item = container_.front();
                container_.pop_front();
            } else {
                typename container_type::iterator base = itr.base();
                item = (*base);
                container_.erase(base);
            }

            return true;
        }

    private:
        size_t bounded_capacity_;
        container_type container_;
        value_comparer comparer_;
    };

    /// @class PriorityComparer
    /// This is the default PriorityContainer comparer.
    /// It expects that the objects being compared have overloaded
    /// < and > operators.
    /// @tparam T The type of objects to compare.
    template<typename T> class PriorityComparer {
    public:
        /// Initializes a new instance of the PriorityComparer<T> class.
        PriorityComparer() {
        }
        /// Compares two objects and returns a value indicating whether one is
        /// less than, equal to, or greater than the other.
        /// Implement this method to provide a customized sort order comparison
        ///  for type T.
        /// @param x The first object to compare.
        /// @param y The second object to compare.
        /// @return A signed integer that indicates the relative values of
        /// x and y, as shown in the following table.
        ///
        /// Value             | Meaning
        /// ------------------|---------------------
        /// Less than zero    | x is less than y.
        /// Zero              | x equals y.
        /// Greater than zero | x is greater than y.
        ///
        int operator() (const T& x, const T& y) const {
            if (x < y)
                return -1;
            else if (x > y)
                return 1;
            else
                return 0;
        }
    };

    template<typename T,
    typename ContainerType = PriorityContainer<T, PriorityComparer<T>>,
    typename ConditionVariableGenerator = StdConditionVariableGenerator>
    class PriorityBlockingCollection : public BlockingCollection<T, ContainerType, ConditionVariableGenerator> {
    public:
        using base = BlockingCollection<T, ContainerType, ConditionVariableGenerator>;

        /// Initializes a new instance of the PriorityBlockingCollection<T>
        /// class without an upper-bound.
        PriorityBlockingCollection()
        : base() {
        }

        /// Initializes a new instance of the PriorityBlockingCollection<T>
        /// class with the specified upper-bound.
        /// @param capacity The bounded size of the collection.
        explicit PriorityBlockingCollection(size_t capacity)
        : base(capacity) {
        }

        // "PriorityBlockingCollection" objects cannot be copied or assigned
        PriorityBlockingCollection(const PriorityBlockingCollection&) = delete;
        PriorityBlockingCollection& operator=(const PriorityBlockingCollection&) = delete;

        /// Removes the lowest priority item from the
        /// PriorityBlockingCollection<T>.
        /// A call to TakePrio may block until an item is available to be
        /// removed.
        /// @param[out] item The lowest priority item removed from the
        /// collection.
        void take_prio(T& item) {
            try_take_prio(item, std::chrono::milliseconds(-1));
        }

        /// Tries to remove the lowest priority item from the
        /// PriorityBlockingCollection<T>.
        /// If the collection is empty, this method immediately returns
        /// immediately without taking an item.
        /// @param[out] item The lowest priority item removed from the
        /// collection.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        BlockingCollectionStatus try_take_prio(T& item) {
            return try_take_prio(item, std::chrono::milliseconds::zero());
        }

        /// Tries to remove the lowest priority item from the
        /// PriorityBlockingCollection<T> in the specified time period.
        /// @param[out] item The lowest priority item removed from the
        /// collection.
        /// @param rel_time An object of type std::chrono::duration
        /// representing the maximum time to spend waiting.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/duration
        template<class Rep, class Period> BlockingCollectionStatus
        try_take_prio(T& item,
        const std::chrono::duration<Rep, Period>& rel_time) {
            {
                std::unique_lock<typename ConditionVariableGenerator::lock_type>
                guard(base::lock());

                auto status = base::wait_not_empty_condition(guard, rel_time);

                if (BlockingCollectionStatus::Ok != status)
                    return status;

                if (!base::container().try_take_prio(item))
                    return BlockingCollectionStatus::InternalError;

                base::signal(base::container().size(), true);
            }
            return BlockingCollectionStatus::Ok;
        }

        /// Takes up to count low priority elements from the
        /// PriorityBlockingCollection<T>.
        /// A call to take_prio_bulk may block until an element is available
        /// to be removed.
        /// Use std::make_move_iterator if the elements should be moved instead
        /// of copied.
        /// @param[out] first Contains first item taken.
        /// @param count The number of elements to take from collection.
        /// @param[out] taken The actual number of elements taken.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template<typename Iterator> BlockingCollectionStatus
        take_prio_bulk(Iterator first, size_t count, size_t& taken) {
            return try_take_prio_bulk(first, count, taken,
            std::chrono::milliseconds(-1));
        }

        /// Takes up to count low priority elements from the
        /// PriorityBlockingCollection<T>.
        /// If the collection is empty, this method immediately returns without
        /// taking any items.
        /// Use std::make_move_iterator if the elements should be moved instead
        /// of copied.
        /// @param[out] first Contains first item taken.
        /// @param count The number of elements to take from collection.
        /// @param[out] taken The actual number of elements taken.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        template<typename Iterator> BlockingCollectionStatus
        try_take_prio_bulk(Iterator first, size_t count, size_t& taken) {
            return try_take_prio_bulk(first, count, taken,
            std::chrono::milliseconds::zero());
        }

        /// Tries to take up to count low priority elements from the
        /// PriorityBlockingCollection<T> within
        /// the specified time period.
        /// If the collection is empty, this method immediately returns without
        /// taking any items.
        /// Use std::make_move_iterator if the elements should be moved instead
        /// of copied.
        /// @param[out] first Contains first item taken.
        /// @param count The number of elements to take from collection.
        /// @param[out] taken The actual number of elements taken.
        /// @return A BlockCollectionStatus code.
        /// @see BlockingCollectionStatus
        /// @see http://en.cppreference.com/w/cpp/chrono/durations
        template <typename Iterator, class Rep, class Period>
        BlockingCollectionStatus try_take_prio_bulk(Iterator first,
        size_t count, size_t& taken,
        const std::chrono::duration<Rep, Period>& rel_time) {
            {
                taken = 0;

                if (count == 0)
                    return BlockingCollectionStatus::Ok;

                std::unique_lock<typename ConditionVariableGenerator::lock_type>
                guard(base::lock());

                auto status = base::wait_not_empty_condition(guard, rel_time);

                if (BlockingCollectionStatus::Ok != status)
                    return status;

                auto end = first + count;

                for (; first != end; ++first)  {
                    if (!base::container().try_take_prio((*first)))
                        break;

                    if (++taken == count)
                        break;
                }
            }
            return BlockingCollectionStatus::Ok;
        }
    };

    namespace detail {
        struct ProducerType {};
        struct ConsumerType {};

        template< typename T >
        struct is_producer : std::false_type { };

        template<>
        struct is_producer<ProducerType> : std::true_type { };

        /// @class Guard
        /// Implements a strictly scope-based BlockingCollection wrapper.
        /// The class Guard is a BlockingCollection wrapper that provides a
        /// convenient RAII-style
        /// mechanism for attaching the current thread as a producer or
        /// consumer to the BlockingCollection for the
        /// duration of the scoped block.
        ///
        /// When a Guard object is created, it attaches the current thread as a
        /// producer or consumer of the
        /// BlockingCollection it is given. When control leaves the scope in
        /// which the Guard object
        /// was created, the Guard is destructed and the current thread is
        /// detached from the BlockingCollection.
        ///
        /// The Guard class makes it simple for threads to register as producer
        /// or consumers with the BlockingCollection<T>
        /// instance. Plus it ensures the thread will be detached from the
        /// BlockingCollection<T> in an
        /// exception scenario.
        ///
        /// The Guard class is non-copyable.
        /// @tparam BlockingCollectionType The type of BlockingCollection to
        /// Guard.
        /// @tparam GuardType The type of Guard to create (i.e. ProducerType
        /// or ConsumerType).
        /// @see ProducerGuard
        /// @see ConsumerGuard
        /// http://en.wikipedia.com/wiki/Resource_Acquisition_Is_Initialization
        template<typename BlockingCollectionType, typename GuardType>
        class Guard {
        public:
            explicit Guard(BlockingCollectionType &collection)
            : collection_(collection) {
                attach_i(is_producer<GuardType>());
            }

            Guard(Guard const&) = delete;
            Guard& operator=(Guard const&) = delete;

            ~Guard() {
                detach_i(is_producer<GuardType>());
            }

        private:
            void attach_i(std::false_type) {
                collection_.attach_consumer();
            }

            void attach_i(std::true_type) {
                collection_.attach_producer();
            }

            void detach_i(std::false_type) {
                collection_.detach_consumer();
            }

            void detach_i(std::true_type) {
                collection_.detach_producer();
            }

            BlockingCollectionType& collection_;
        };
    } // namespace detail

    /// A type alias for Guard<T, ProducerType>
    template<typename T>
    using ProducerGuard = detail::Guard<T, detail::ProducerType>;

    /// A type alias for Guard<T, ConsumerType>
    template<typename T>
    using ConsumerGuard = detail::Guard<T, detail::ConsumerType>;

    /// A type alias for BlockingCollection<T, Stack> - a last in-first out
    /// (LIFO) BlockingCollection.
    template<typename T>
    using BlockingStack = BlockingCollection<T, StackContainer<T>>;

    /// A type alias for BlockingCollection<T, Queue> - a first in-first out
    /// (FIFO) BlockingCollection.
    template<typename T>
    using BlockingQueue = BlockingCollection<T, QueueContainer<T>>;

    /// A type alias for BlockingCollection<T, PriorityQueue> - a priority-based
    /// BlockingCollection.
    template<typename T>
    using BlockingPriorityQueue = BlockingCollection<T,
    PriorityContainer<T, PriorityComparer<T>>>;

#ifdef _WIN32
    /// @class WIN32_CRITICAL_SECTION
    /// WIN32_CRITICAL_SECTION wraps the Win32 CRITICAL_SECTION object so that
    /// it meets the BasicLockable requirement (i.e. lock and unlock member
    /// functions).
    ///
    /// @see WIN32_SRWLOCK
    class WIN32_CRITICAL_SECTION {
    public:
        WIN32_CRITICAL_SECTION() {
            InitializeCriticalSection(&cs_);
        }

        void lock() {
            EnterCriticalSection(&cs_);
        }

        void unlock() {
            LeaveCriticalSection(&cs_);
        }

        CRITICAL_SECTION& native_handle() {
            return cs_;
        }
    private:
        CRITICAL_SECTION cs_;
    };

    /// @class WIN32_SRWLOCK
    /// WIN32_SRWLOCK wraps the Win32 SRWLOCK object so that it meets the
    /// BasicLockable requirement (i.e. lock and unlock member functions).
    ///
    /// @see WIN32_CRITICAL_SECTION
    class WIN32_SRWLOCK {
    public:
        WIN32_SRWLOCK() {
            InitializeSRWLock(&srw_);
        }

        void lock() {
            AcquireSRWLockExclusive(&srw_);
        }

        void unlock() {
            ReleaseSRWLockExclusive(&srw_);
        }

        SRWLOCK& native_handle() {
            return srw_;
        }
    private:
        SRWLOCK srw_;
    };

    template <>
    struct ConditionVarTraits<CONDITION_VARIABLE, WIN32_SRWLOCK> {
        static void initialize(CONDITION_VARIABLE& cond_var) {
            InitializeConditionVariable(&cond_var);
        }

        static void signal(CONDITION_VARIABLE& cond_var) {
            WakeConditionVariable(&cond_var);
        }

        static void broadcast(CONDITION_VARIABLE& cond_var) {
            WakeAllConditionVariable(&cond_var);
        }

        static void wait(CONDITION_VARIABLE& cond_var,
        std::unique_lock<WIN32_SRWLOCK>& lock) {
            SleepConditionVariableSRW(&cond_var, &lock.mutex()->native_handle(),
            INFINITE, 0);
        }

        template<class Rep, class Period> static bool wait_for(
            CONDITION_VARIABLE& cond_var, std::unique_lock<WIN32_SRWLOCK>& lock,
            const std::chrono::duration<Rep, Period>& rel_time) {
            DWORD milliseconds = static_cast<DWORD>(rel_time.count());

            if (!SleepConditionVariableSRW(&cond_var,
            &lock.mutex()->native_handle(), milliseconds, 0)) {
                if (GetLastError() == ERROR_TIMEOUT)
                    return true;
            }

            return false;
        }
    };

    template <>
    struct ConditionVarTraits<CONDITION_VARIABLE, WIN32_CRITICAL_SECTION> {
        static void initialize(CONDITION_VARIABLE& cond_var) {
            InitializeConditionVariable(&cond_var);
        }

        static void signal(CONDITION_VARIABLE& cond_var) {
            WakeConditionVariable(&cond_var);
        }

        static void broadcast(CONDITION_VARIABLE& cond_var) {
            WakeAllConditionVariable(&cond_var);
        }

        static void wait(CONDITION_VARIABLE& cond_var,
        std::unique_lock<WIN32_CRITICAL_SECTION>& lock) {
            SleepConditionVariableCS(&cond_var, &lock.mutex()->native_handle(),
            INFINITE);
        }

        template<class Rep, class Period> static bool wait_for(
            CONDITION_VARIABLE& cond_var,
            std::unique_lock<WIN32_CRITICAL_SECTION>& lock,
            const std::chrono::duration<Rep, Period>& rel_time) {
            DWORD milliseconds = static_cast<DWORD>(rel_time.count());

            if (!SleepConditionVariableCS(&cond_var,
            &lock.mutex()->native_handle(), milliseconds)) {
                if (GetLastError() == ERROR_TIMEOUT)
                    return true;
            }

            return false;
        }
    };

    using Win32ConditionVariableGenerator = ConditionVariableGenerator<
    ThreadContainer<std::thread::id>, NotFullSignalStrategy<16>,
    NotEmptySignalStrategy<16>, CONDITION_VARIABLE, WIN32_SRWLOCK>;
#endif
} // namespace code_machina

#endif /* BlockingCollection_h */
