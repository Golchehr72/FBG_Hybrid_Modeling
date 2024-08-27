#!/usr/bin/env python3

reachedThreatNum_Synchronization = 0
reachedThreatNum_StopEvent = 0

def costumBarrier_Synchronization(threadCondition, targetThreadNum_Synchronization):
    # For synchronization, we only need to synchronize two stereo cameras to capture images at the same time by frequency of 15 Hz
    # The FBG interrogator store data by frequency of 100 Hz
    global reachedThreatNum_Synchronization
    with threadCondition:
        reachedThreatNum_Synchronization += 1
        #print('reachedThreatNum: ', reachedThreatNum_Synchronization)
        if reachedThreatNum_Synchronization == targetThreadNum_Synchronization:
            threadCondition.notify_all()
        else:
            while reachedThreatNum_Synchronization < targetThreadNum_Synchronization:
                #print('reachedThreatNum in while:',reachedThreatNum_Synchronization)
                threadCondition.wait()
                #itr +=1
                #print('itr:',itr)
                #if itr == 100:
                    #print('in 20')
                    #threadCondition.notify_all()
                    #break
            #itr = 0
            reachedThreatNum_Synchronization = 0
            #print('out of else')

def costumBarrier_StopEvent(threadLock, targetThreadNum_StopEvent, stopEvent):
    #reachedThreatNum = 0
    global reachedThreatNum_StopEvent
    

    with threadLock:
        reachedThreatNum_StopEvent += 1
        #print('reachedThreatNum_StopEvent: ', reachedThreatNum_StopEvent)
        if reachedThreatNum_StopEvent == targetThreadNum_StopEvent:
            stopEvent.set()  # Signal that barrier is reached
        else:
            while not stopEvent.is_set():
                #print('reachedThreatNum_StopEvent in while:',reachedThreatNum_StopEvent)
                threadLock.release()  # Release lock before waiting
                stopEvent.wait()
                threadLock.acquire()  # Re-acquire lock after waiting
    