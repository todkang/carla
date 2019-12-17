// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>

namespace carla {
namespace traffic_manager {

  /// This class is the template for messaging functionality between
  /// different stage classes to send and receive data.
  /// One object of this type can only facilitate receiving data from
  /// a sender stage and passing the data onto a receiver stage.
  /// The class maintains state internally and blocks send or receive
  /// requests until data is available/successfully passed on.
  template <typename Data>
  class Messenger {

  private:

    /// Flag used to wake up and join any waiting function calls on this object.
    std::atomic<bool> stop_messenger;
    /// Member used to hold data sent by the sender.
    std::queue<Data> data;
    /// Mutex used to manage contention between the sender and receiver.
    std::mutex data_modification_mutex;
    /// Variable to conditionally block receiver in case there is no data yet.
    std::condition_variable condition_variable;

    int64_t data_size;

  public:

    Messenger() : data_size(0) {
      stop_messenger.store(false);
    }
    ~Messenger() {}

    /// This method receives data from a sender, stores in a member and
    /// increments state.
    void SendData(Data packet) {
      if(!stop_messenger.load()){
        return;
      }

      std::unique_lock<std::mutex> lock(data_modification_mutex);
      data.push(packet);
      data_size++;
      lock.unlock();

      condition_variable.notify_one();
    }

    /// This method presents stored data to the receiver and increments state.
    void ReceiveData(Data *packet) {
      if(!stop_messenger.load()){
        packet = nullptr;
        return;
      }

      std::unique_lock<std::mutex> lock(data_modification_mutex);
      condition_variable.wait(lock, [&] {return data_size > 0;});

      *packet = data.front();
      data.pop();
      data_size--;

    }

    /// This method returns the current value of the state counter.
    int GetState() {
      return state_counter.load();
    }

    /// This method unblocks any waiting calls on this object.
    void Stop() {
      stop_messenger.store(true);
    }

    /// This method restores regular functionality of the messenger.
    /// This method needs to be called if the messenger has to be
    /// used again after a call to the Stop() method.
    void Start() {
      stop_messenger.store(false);
    }

  };

} // namespace traffic_manager
} // namespace carla
