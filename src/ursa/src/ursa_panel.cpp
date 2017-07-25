/*********************************************************************
 * Copyright LAOSAAC 2017
 *
 * Author: LD
 * GUI Panel for URSA drone control in RVIZ
 *********************************************************************/

#include <ursa_panel.h>
#include <stdio.h>
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QLabel>
#include "ursa/TakeoffLand.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ursa_gui::ursa_panel, rviz::Panel)

namespace ursa_gui{

ursa_panel::ursa_panel(QWidget* parent ):
	rviz::Panel(parent),
	_altitude(1.0f) {

	//Takeoff/land buttons
	QHBoxLayout* takeoff_layout = new QHBoxLayout;
	_takeoff_button = new QPushButton;
	takeoff_layout->addWidget( _takeoff_button );
	_takeoff_button->setText("Takeoff");
	_land_button = new QPushButton;
	takeoff_layout->addWidget( _land_button );
	_land_button->setText("Land");

	//Altitude setpoint
	QHBoxLayout* altitude_layout = new QHBoxLayout;
	altitude_layout->addWidget( new QLabel( "Target altitude (m):" ));
	_altitude_editor = new QLineEdit;
	_altitude_editor->setText(QString::number(_altitude));
	altitude_layout->addWidget(_altitude_editor);

	//Container for all
	QVBoxLayout* layout = new QVBoxLayout;
	layout->addLayout( takeoff_layout );
	layout->addLayout( altitude_layout );
	setLayout( layout );

	//Connect sigs and slots
	connect( _takeoff_button, SIGNAL( clicked() ), this, SLOT( takeoff() ));
	connect( _land_button, SIGNAL( clicked() ), this, SLOT( land() ));

	//Setup service for comms with python node
	_takeoff_service = _n.serviceClient<ursa::TakeoffLand>("ursa_takeoff_land");
}

// Takeoff event
void ursa_panel::takeoff(){
	ROS_INFO("Attempting take-off...\n");
	ursa::TakeoffLand srv;
	srv.request.takeoff = 1;
	QString heightInput=_altitude_editor->text();
	srv.request.height = heightInput.toDouble();
	if (_takeoff_service.call(srv)) {
		ROS_INFO("Success!");
	} else {
		ROS_ERROR("Failed to call service!");
	}
}

// Land event
void ursa_panel::land(){
	ROS_INFO("Attempting Landing...\n");
	ursa::TakeoffLand srv;
	srv.request.takeoff = 0;
	if (_takeoff_service.call(srv)) {
		ROS_INFO("Success!");
	} else {
		ROS_ERROR("Failed to call service!");
	}
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ursa_panel::save( rviz::Config config ) const
{
	rviz::Panel::save( config );
  // config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void ursa_panel::load( const rviz::Config& config )
{
	rviz::Panel::load( config );
  // QString topic;
  // if( config.mapGetString( "Topic", &topic ))
  // {
  //   output_topic_editor_->setText( topic );
  //   updateTopic();
  // }
}

} // END namespace ursa_gui