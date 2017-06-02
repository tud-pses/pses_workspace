#ifndef PARSINGTHREAD_H
#define PARSINGTHREAD_H

#include <pses_basis/communicationthread.h>
#include <pses_basis/base64Decoder.h>

class ParsingThread : public CommunicationThread{
public:
  ParsingThread(){

  }
  inline void startThread(){
    active = true;
    worker = std::thread(&ReadingThread::workerFunction, this);
  }

  inline void stopThread(){
    active = false;
    wakeUp();
    worker.join();
  }

  void workerFunction();
};

#endif // PARSINGTHREAD_H
