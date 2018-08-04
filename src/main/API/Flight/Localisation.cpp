/*
 * Localisation.cpp
 *
 *  Created on: 09-May-2018
 *      Author: User
 */




#include "command/localisationCommand.h"
#include "flight/posEstimate.h"
#include "API/API-Utils.h"


#include "Localisation.h"



void Localisation_P::init(localisation_type localisation)
{

localisationType=localisation;

}


void Localisation_P::setCommand(localisation_cmd_type command)
{


	if(command!=LC_GO_TO_WAY_POINT)
	{

	cmd_def cmd= {(cmd_type)command,0,0,0};



    command_add(cmd);

	}

}


void Localisation_P::setCommand(localisation_cmd_type command, int16_t desiredX, int16_t desiredY)
{


	if(command==LC_GO_TO_WAY_POINT)
	{

	cmd_def cmd= {(cmd_type)command,desiredX,desiredY,0};


	command_add(cmd);

	}

}


void Localisation_P::setCommand(localisation_cmd_type command, int16_t desiredX, int16_t desiredY, int16_t desiredZ)
{


	if(command==LC_GO_TO_WAY_POINT)
		{

		cmd_def cmd= {(cmd_type)command,desiredX,desiredY,desiredZ};


		command_add(cmd);

		}


}


void  Localisation_P::startLocalisation()
{


	isLocalisationOn=true;


}





void  Localisation_P::stopLocalisation()
{


	isLocalisationOn=false;


}


Localisation_P Localisation;

