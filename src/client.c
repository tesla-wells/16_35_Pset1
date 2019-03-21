#include "client.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>

struct hostent *he;
struct sockaddr_in their_addr; /* connector's address information */
int sockfd, numbytes;


void send_vehicles(int n_vehicles, vehicle * vehicles){
    // create the posx,posy,theta arrays
    double * pos_x = malloc(n_vehicles * sizeof(double));
    double * pos_y = malloc(n_vehicles * sizeof(double));
    double * theta= malloc(n_vehicles * sizeof(double));
    int i =0;
    for (vehicle * v = vehicles; v < vehicles + n_vehicles; v++) {
        pos_x[i] = v->position[0]; pos_y[i] = v->position[1]; theta[i] = v->position[2]; i++;
    }
    // send the data
    send_vehicle_data(n_vehicles,pos_x,pos_y,theta);
    free(pos_x);
    free(pos_y);
    free(theta);
}

void send_vehicle_data(int n_vehicles, double * pos_x, double * pos_y, double * theta){
    // construct pickle datagram
    // fyi: if you're reading this, this isn't normally how one would construct a datagram.
    //      this is because the display server is written in python and cpickle was used.
    //      we'll likely cover how to do this properly later in the course --ckf
    char data [1024];
    data[0] = '\0'; // null terminator for string
    // print number of vehicles to string
    sprintf(data,"(I%i\n",n_vehicles);
    sprintf(data,"%s(lp1\n",data);
    sprintf(data,"%sF%f\n",data,pos_x[0]);
    for (int i = 1; i < n_vehicles;i++) sprintf(data,"%saF%f\n",data,pos_x[i]);
    sprintf(data,"%sa(lp2\n",data);
    sprintf(data,"%sF%f\n",data,pos_y[0]);
    for (int i = 1; i < n_vehicles;i++) sprintf(data,"%saF%f\n",data,pos_y[i]);
    sprintf(data,"%sa(lp3\n",data);
    sprintf(data,"%sF%f\n",data,theta[0]);
    for (int i = 1; i < n_vehicles;i++) sprintf(data,"%saF%f\n",data,theta[i]);
    sprintf(data,"%satp4\n.",data);
    send_data(data);
}

void open_server(char * ip, int port){
    if ((he=gethostbyname(ip)) == NULL) {  /* get the host info */
        herror("gethostbyname");
        exit(1);
    }
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }
    their_addr.sin_family = AF_INET;      /* host byte order */
    their_addr.sin_port = htons(PORTNUM);    /* short, network byte order */
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);
    bzero(&(their_addr.sin_zero), 8);     /* zero the rest of the struct */

    if (connect(sockfd, (struct sockaddr *)&their_addr, \
                                          sizeof(struct sockaddr)) == -1) {
        perror("connect");
        exit(1);
    }
}

void send_data(char * data){
    if (send(sockfd, data, strlen(data), 0) == -1){
        perror("send");
        exit(1);
    }
}

void close_server(){
    close(sockfd);
}

pid_t child;
void launch_display_server(){
    int fd[2];
    pipe(fd); // for stdout
    // fork into a subprocess
    child = fork();
    if (child == (pid_t)-1) {
        // error! probably out of resources;
        exit(1);
    }
    if (!child) {
        execlp("/usr/bin/python2.7","python2.7","../python/DisplayServer.py",NULL);
        exit(-1);
    }

}
void close_display_server(){
    if (child) kill(child,SIGKILL);
}
