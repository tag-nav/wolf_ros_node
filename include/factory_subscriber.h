//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
#ifndef FACTORY_SUBSCRIBER_H_
#define FACTORY_SUBSCRIBER_H_

// wolf
#include <core/common/factory.h>
#include <core/utils/params_server.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// #include "wolf_ros_subscriber.h"
// std


namespace wolf
{
/** \brief Processor factory class
 *
 * This factory can create objects of classes deriving from ProcessorBase.
 *
 * Specific object creation is invoked by create(TYPE, params), and the TYPE of processor is identified with a string.
 * For example, the following processor types are implemented,
 *   - "ODOM 3d" for ProcessorOdom3d
 *   - "ODOM 2d" for ProcessorOdom2d
 *   - "GPS"     for ProcessorGPS
 *
 * The rule to make new TYPE strings unique is that you skip the prefix 'Processor' from your class name,
 * and you build a string in CAPITALS with space separators.
 *   - ProcessorImageFeature -> ````"IMAGE"````
 *   - ProcessorLaser2d -> ````"LASER 2d"````
 *   - etc.
 *
 * The methods to create specific processors are called __creators__.
 * Creators must be registered to the factory before they can be invoked for processor creation.
 *
 * This documentation shows you how to:
 *   - Access the Factory
 *   - Register and unregister creators
 *   - Create processors
 *   - Write a processor creator for ProcessorOdom2d (example).
 *
 * #### Accessing the Factory
 * The FactoryProcessor class is a singleton: it can only exist once in your application.
 * To obtain an instance of it, use the static method get(),
 *
 *     \code
 *     FactoryProcessor::get()
 *     \endcode
 *
 * You can then call the methods you like, e.g. to create a processor, you type:
 *
 *     \code
 *     FactoryProcessor::create(...); // see below for creating processors ...
 *     \endcode
 *
 * #### Registering processor creators
 * Prior to invoking the creation of a processor of a particular type,
 * you must register the creator for this type into the factory.
 *
 * Registering processor creators into the factory is done through registerCreator().
 * You provide a processor type string (above), and a pointer to a static method
 * that knows how to create your specific processor, e.g.:
 *
 *     \code
 *     FactoryProcessor::registerCreator("ODOM 2d", ProcessorOdom2d::create);
 *     \endcode
 *
 * The method ProcessorOdom2d::create() exists in the ProcessorOdom2d class as a static method.
 * All these ProcessorXxx::create() methods need to have exactly the same API, regardless of the processor type.
 * This API includes a processor name, and a pointer to a base struct of parameters, ParamsProcessorBasePtr,
 * that can be derived for each derived processor.
 *
 * Here is an example of ProcessorOdom2d::create() extracted from processor_odom_2d.h:
 *
 *     \code
 *     static ProcessorBasePtr create(const std::string& _name, ParamsProcessorBasePtr _params)
 *     {
 *         // cast _params to good type
 *         ParamsProcessorOdom2d* params = (ParamsProcessorOdom2d*)_params;
 *
 *         ProcessorBasePtr prc = new ProcessorOdom2d(params);
 *         prc->setName(_name); // pass the name to the created ProcessorOdom2d.
 *         return prc;
 *     }
 *     \endcode
 *
 * #### Achieving automatic registration
 * Currently, registering is performed in each specific ProcessorXxxx source file, processor_xxxx.cpp.
 * For example, in processor_odom_2d.cpp we find the line:
 *
 *     \code
 *     const bool registered_odom_2d = FactoryProcessor::registerCreator("ODOM 2d", ProcessorOdom2d::create);
 *     \endcode
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the ProcessorOdom2d class).
 * Therefore, at application level, all processors that have a .cpp file compiled are automatically registered.
 *
 * #### Unregister processor creators
 * The method unregisterCreator() unregisters the ProcessorXxx::create() method.
 * It only needs to be passed the string of the processor type.
 *
 *     \code
 *     FactoryProcessor::unregisterCreator("ODOM 2d");
 *     \endcode
 *
 * #### Creating processors
 * Prior to invoking the creation of a processor of a particular type,
 * you must register the creator for this type into the factory.
 *
 * To create a ProcessorOdom2d, you type:
 *
 *     \code
 *     FactoryProcessor::create("ODOM 2d", "main odometry", params_ptr);
 *     \endcode
 *
 * #### Example 1 : using the Factories alone
 * We provide the necessary steps to create a processor of class ProcessorOdom2d in our application,
 * and bind it to a SensorOdom2d:
 *
 *     \code
 *     #include "sensor_odom_2d.h"      // provides SensorOdom2d    and FactorySensor
 *     #include "processor_odom_2d.h"   // provides ProcessorOdom2d and FactoryProcessor
 *
 *     // Note: SensorOdom2d::create()    is already registered, automatically.
 *     // Note: ProcessorOdom2d::create() is already registered, automatically.
 *
 *     // First create the sensor (See FactorySensor for details)
 *     SensorBasePtr sensor_ptr = FactorySensor::create ( "ODOM 2d" , "Main odometer" , extrinsics , &intrinsics );
 *
 *     // To create a odometry integrator, provide a type="ODOM 2d", a name="main odometry", and a pointer to the parameters struct:
 *
 *     ParamsProcessorOdom2d  params({...});   // fill in the derived struct (note: ProcessorOdom2d actually has no input params)
 *
 *     ProcessorBasePtr processor_ptr =
 *         FactoryProcessor::create ( "ODOM 2d" , "main odometry" , &params );
 *
 *     // Bind processor to sensor
 *     sensor_ptr->addProcessor(processor_ptr);
 *     \endcode
 *
 * #### Example 2: Using the helper API in class Problem
 * The WOLF uppermost node, Problem, makes the creation of sensors and processors, and the binding between them, even simpler.
 *
 * The creation is basically replicating the factories' API. The binding is accomplished by passing the sensor name to the Processor installer.
 *
 * The example 1 above can be accomplished as follows (we obviated for simplicity all the parameter creation),
 *
 *     \code
 *     #include "sensor_odom_2d.h"
 *     #include "processor_odom_2d.h"
 *     #include "problem.h"
 *
 *     Problem problem(FRM_PO_2d);
 *     problem.installSensor    ( "ODOM 2d" , "Main odometer" , extrinsics      , &intrinsics );
 *     problem.installProcessor ( "ODOM 2d" , "Odometry"      , "Main odometer" , &params     );
 *     \endcode
 *
 * You can also check the code in the example file ````src/examples/test_wolf_factories.cpp````.
 */
    class Subscriber;
    typedef Factory<Subscriber,
                    const std::string&,
                    const ParamsServer&,
                    const SensorBasePtr,
                    ros::NodeHandle&> FactorySubscriber;
template<>
inline std::string FactorySubscriber::getClass() const
{
  return "FactorySubscriber";
}


#define WOLF_REGISTER_SUBSCRIBER(SubscriberType)                        \
    namespace{ const bool WOLF_UNUSED SubscriberType##Registered =      \
            wolf::FactorySubscriber::registerCreator(#SubscriberType, SubscriberType::create); } \

} /* namespace wolf */
#endif /* FACTORY_SUBSCRIBER_H_ */
