/*
 * htond.h
 *
 *  Created on: Jan 29, 2018
 *      Author: duser
 */

#ifndef HTOND_H_
#define HTOND_H_


//Converts 8 bytes from host byte order to network byte order
double htond(double host)
{
	static int is_big = -1;			//1 if Big Endian 0 if Little Endian
	double network;							//Holds network byte order representation
	double host1 = host;
	char *n = (char *)&network + sizeof(double)-1;
	char *h = (char *)&host1;
	unsigned int x = 1;
	char *y = (char *)&x;
	char z = 1;
	int i;

	if(is_big < 0)
	{
		if(*y == z)
		{
			is_big = 0;
		}
		else
		{
			is_big = 1;
		}
	}

	if(is_big)
	{
		return host;
	}

	for(i=0;i<sizeof(double);i++)
	{
		*n = *h;
		n--;
		h++;
	}
	return network;
}

//Converts 8 bytes from network byte order to host byte order
double ntohd(double network)
{
	static int is_big = -1;			//1 if Big Endian 0 if Little Endian
	double host;								//Holds host byte order representation
	char *n = (char *)&network + sizeof(double) -1;
	char *h = (char *)&host;
	unsigned int x = 1;
	char *y = (char *)&x;
	char z = 1;
	int i;

	if(is_big < 0)
	{
		if(*y == z)
		{
			is_big = 0;
		}
		else
		{
			is_big = 1;
		}
	}

	if(is_big)
	{
		return network;
	}

	for(i=0;i<sizeof(double);i++)
	{
		*h = *n;
		n--;
		h++;
	}
	return host;
}




#endif /* HTOND_H_ */
