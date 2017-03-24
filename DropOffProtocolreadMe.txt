line 2485 void dropProtocolHandler(const std_msgs::String& msg)		//dropOffProtocol
{
	waitToDrop = true;

    if(waitToDrop == true && isDroppingOff == true && targetCollected == true)	//allow to continue	//dropOffProtocol
    {
	std_msgs::String msg;
        msg.data = "allowed to proceed";
        infoLogPublisher.publish(msg);
    }

    if(waitToDrop == true && isDroppingOff == false && targetCollected == true)  // stop movement here, resume when waitToDrop = false
    {
 	std_msgs::String msg;
        msg.data = "I should be waiting"; 
        infoLogPublisher.publish(msg);
    }

    if(waitToDrop == true && targetCollected == false) // allow to continue, but notified swarmie at center
    {
 	std_msgs::String msg;
        msg.data = "I don't care.";
        infoLogPublisher.publish(msg);
    }
}

void finishedProtocolHandler(const std_msgs::String& msg) //give instruction if have block, waitToDrop = true
{
	waitToDrop = false;

	std_msgs::String msgd;
        msgd.data = "waitToDrop turned to false";
        infoLogPublisher.publish(msgd);
}
