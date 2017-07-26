/*********************************************************************
 * Copyright LAOSAAC 2017
 *
 * Author: LD
 * GUI Panel for URSA drone control in RVIZ
 *********************************************************************/
#include <ros/ros.h>
#include <rviz/panel.h>

#ifndef URSA_PANEL_H_
#define URSA_PANEL_H_

class QPushButton;
class QLineEdit;

namespace ursa_gui{

class ursa_panel: public rviz::Panel {
Q_OBJECT
public:
	ursa_panel(QWidget* parent = 0);
	virtual void load( const rviz::Config& config );
	virtual void save( rviz::Config config ) const;

//public Q_SLOTS:

protected Q_SLOTS:
	//Slot for taking off
	void takeoff(); 
	//Slot for landing
	void land(); 

protected:
	//Button to take off
	QPushButton* _takeoff_button;

	//Button to land
	QPushButton* _land_button;

  	// One-line text editor for entering the altitude
  	QLineEdit* _altitude_editor;
  	double _altitude;

  	// Handle to the ros node and service
  	ros::NodeHandle _n;
  	ros::ServiceClient _takeoff_service;


};//End class ursa_panel


}//End namespace ursa_gui


#endif