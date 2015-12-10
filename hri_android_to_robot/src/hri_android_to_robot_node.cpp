
#include <ros/ros.h>			/* ros functions 	*/
#include "std_msgs/String.h"
#include <signal.h>
#include <stdio.h>
#include <cstdlib>
#include <cstring>

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */

#include <errno.h>
#include <ctype.h>

/*  Global constants  */

#define ECHO_PORT          (5565)   /* Default port number  */
#define MAX_LINE           (4096)   /* Maximum size for buffer */
#define LISTENQ        	   (1024)   /*  Backlog for listen()   */



class DroidToBotClass
{
	public:
  		DroidToBotClass();
		void publishMsg(char*);
	private:
  		ros::NodeHandle n;
		ros::Publisher pub;
};


ssize_t Readline(int, char*, size_t);
ssize_t Writeline(int, const char*, size_t);

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "android_to_robot");
    ros::NodeHandle nh;
  
    int       list_s;                /*  listening socket          */
    int       conn_s;                /*  connection socket         */
    short int port;                  /*  port number               */
    struct    sockaddr_in servaddr;  /*  socket address structure  */
    char      buffer[MAX_LINE];      /*  character buffer          */
    char     *endptr;                /*  for strtol()              */
    char      prntMsg[5020];			

  
    /*  Get port number from the command line, and
        set to default port if no arguments were supplied  */

    if ( argc == 2 ) 
    {
	port = strtol(argv[1], &endptr, 0);
	if ( *endptr ) 
	{
	    fprintf(stderr, "ECHOSERV: Invalid port number.\n");
	    exit(EXIT_FAILURE);
	}
    }
    else if ( argc < 2 ) 
    {
	port = ECHO_PORT;
    }
    else 
    {
	fprintf(stderr, "ECHOSERV: Invalid arguments.\n");
	exit(EXIT_FAILURE);
    }

	
    /*  Create the listening socket  */
;
    if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) 
    {
	fprintf(stderr, "ECHOSERV: Error creating listening socket.\n");
	exit(EXIT_FAILURE);
    }


    /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons(port);

    /*  Bind our socket addresss to the 
	listening socket, and call listen()  */

    if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0) 
    {
	servaddr.sin_port = htons(++port);
	if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
	{
	    servaddr.sin_port = htons(++port);
	    if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
	    {
		servaddr.sin_port = htons(++port);
	        if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
	        {
		    servaddr.sin_port = htons(++port);
	            if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0)
	            {
		      fprintf(stderr, "ECHOSERV: Error calling bind()\n");
		      exit(EXIT_FAILURE);
		    }
                }
            }
	}
    }

    printf("connected on port #: %i " , (int)port );


    if ( listen(list_s, LISTENQ) < 0 ) 
    {
	fprintf(stderr, "ECHOSERV: Error calling listen()\n");
	exit(EXIT_FAILURE);
    }

    
    /*  Enter an infinite loop to respond
        to client requests and echo input  */
    ros::Publisher pubout = nh.advertise<std_msgs::String>("recognizer/output", 1000);

    while ( 1 ) 
    {

	/*  Wait for a connection, then accept() it  */

	if ( (conn_s = accept(list_s, NULL, NULL) ) < 0 ) 
	{
	    fprintf(stderr, "ECHOSERV: Error calling accept()\n");
	    exit(EXIT_FAILURE);
	}


	/* Initialize ROS node */

	//ros::init(argc, argv, "Droid_Bot_server");
  	DroidToBotClass droid_bot;


	/*  Get input line from connected socket   */
        Readline(conn_s, buffer, MAX_LINE-1);
	

	while((strncmp(buffer,"quit",4)!=0 || (strlen(buffer)>6)) && (strlen(buffer)>2) )
	{
  	   droid_bot.publishMsg(buffer);
	   std::string str(buffer);
	   std_msgs::String bufMsg;
	   bufMsg.data = str;
	   pubout.publish(bufMsg);
		
	   strcpy(prntMsg, "Message Recieved: ");
	   strcat(prntMsg, buffer);
	   printf( "%s", prntMsg);
	   
	   Writeline(conn_s, "Got it!\n", strlen("Got it!\n"));
	   strcpy(buffer,"");
	   Readline(conn_s, buffer, MAX_LINE-1);	
	}
	  
	/*  Close the connected socket  */

	if ( close(conn_s) < 0 ) 
	{
	
	    fprintf(stderr, "ECHOSERV: Error calling close()\n");
	    exit(EXIT_FAILURE);
	}
    }


  return(0);
}


/* ********************************************************** */

/* Read a line from socket */

ssize_t Readline(int sockd, char *vptr, size_t maxlen) 
{
    ssize_t n, rc;
    char    c, *buffer;

    buffer = vptr;

    for ( n = 1; n < maxlen; n++ ) 
    {
	
	if ( (rc = read(sockd, &c, 1)) == 1 )  /* read returns # of bytes read */
	{
	    *buffer++ = c;
	    if ( c == '\n' )		       /* quit reading at eol */
		break;
	}
	else if ( rc == 0 ) 
	{
	    if ( n == 1 )
		return 0;
	    else
		break;
	}
	else 
	{
	    if ( errno == EINTR )
		continue;
	    return -1;
	}
    }

    *buffer = 0;
    return n;
}

/* ********************************************************** */

/*  Write a line to socket */ 

ssize_t Writeline(int sockd, const char *vptr, size_t n) 
{
    size_t      nleft;
    ssize_t     nwritten;  /* number  of  bytes  actually written */
    const char *buffer;

    buffer = vptr;
    nleft  = n;

    while ( nleft > 0 ) 
    {
	if ( (nwritten = write(sockd, buffer, nleft)) <= 0 ) 
	{
	    if ( errno == EINTR )
		nwritten = 0;
	    else
		return -1;
	}
	nleft  -= nwritten;
	buffer += nwritten;
    }

    return n;
}


/* ********************************************************* */

/* *******CLASS FUNCTIONS *********** */
DroidToBotClass::DroidToBotClass()
{
   //ros::Subscriber textSub = n.subscribe("recognizer/output", 1000, ListenCallbackCMDStr); std_msgs::String
  pub = n.advertise<std_msgs::String>("recognizer/output", 1000);
}

/* ********************************** */


void DroidToBotClass::publishMsg(char *inString)
{
 
    std_msgs::String msg;

    msg.data = inString;	

    pub.publish(msg);    
 
    return;
 

}
/* *******END CLASS FUNCTIONS *********** */
