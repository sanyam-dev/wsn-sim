# Wireless Sensor Network simulator for model simulations



I am running in to the need of a network simulator which can be simulated to as close as to a real network, and generate data. The framework will take in the network data using some ways of input, which will be developed on the way. The framework should be able to collect data about node to node latency on every message transmitted, with a message id, and a control over the message data. Such level of intricacy will help us simulate a network model as close to specific complex testcases.

So, we need to have a message object, which will have a message id, each message id will be associated to some message transmission metrics such as latency of transfer, sender node, reciever node, message size and the amount of energy consumed each time it was sent.

Now to improve efficiency and space complexity of the framework, instead of creating a new message between every two nodes, every time a message is relayed, instead let's just create a new message object every time the message is generated, and if it is being transmitted from one node to another, we can then have a list of sender nodes, and reciever nodes, transmission count and also the data of the generator node. That way, we can have detailed data about the transmission history of the network, including the amount of data generated, transferred, and utilised.

That might be a lot of data, and we would need to store it on a database, so we will be connecting it to a server which we will host on some cloud storage, we can simulate some local storage for now, but as we keep scaling we will need to create a cloud connection and run the server there. 

AND the most important feature of all this is to be able to use customised protocols to compare network performance,
We need to figure out a way to input the data transmission protocol to simulate the node behaviour. We might need to write some extra code to add some default libraries with a layer of abstraction maybe to provide the customisation which can be done later. Or some way to import the file via an arguement and use the function directly.




