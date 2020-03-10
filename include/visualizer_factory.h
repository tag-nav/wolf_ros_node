#ifndef VISUALIZER_FACTORY_H_
#define VISUALIZER_FACTORY_H_

// wolf
#include <core/common/factory.h>
#include <core/utils/params_server.hpp>
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
 *   - "ODOM 3D" for ProcessorOdom3D
 *   - "ODOM 2D" for ProcessorOdom2D
 *   - "GPS"     for ProcessorGPS
 *
 * The rule to make new TYPE strings unique is that you skip the prefix 'Processor' from your class name,
 * and you build a string in CAPITALS with space separators.
 *   - ProcessorImageFeature -> ````"IMAGE"````
 *   - ProcessorLaser2D -> ````"LASER 2D"````
 *   - etc.
 *
 * The methods to create specific processors are called __creators__.
 * Creators must be registered to the factory before they can be invoked for processor creation.
 *
 * This documentation shows you how to:
 *   - Access the Factory
 *   - Register and unregister creators
 *   - Create processors
 *   - Write a processor creator for ProcessorOdom2D (example).
 *
 * #### Accessing the Factory
 * The ProcessorFactory class is a singleton: it can only exist once in your application.
 * To obtain an instance of it, use the static method get(),
 *
 *     \code
 *     ProcessorFactory::get()
 *     \endcode
 *
 * You can then call the methods you like, e.g. to create a processor, you type:
 *
 *     \code
 *     ProcessorFactory::get().create(...); // see below for creating processors ...
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
 *     ProcessorFactory::get().registerCreator("ODOM 2D", ProcessorOdom2D::create);
 *     \endcode
 *
 * The method ProcessorOdom2D::create() exists in the ProcessorOdom2D class as a static method.
 * All these ProcessorXxx::create() methods need to have exactly the same API, regardless of the processor type.
 * This API includes a processor name, and a pointer to a base struct of parameters, ProcessorParamsBasePtr,
 * that can be derived for each derived processor.
 *
 * Here is an example of ProcessorOdom2D::create() extracted from processor_odom_2D.h:
 *
 *     \code
 *     static ProcessorBasePtr create(const std::string& _name, ProcessorParamsBasePtr _params)
 *     {
 *         // cast _params to good type
 *         ProcessorParamsOdom2D* params = (ProcessorParamsOdom2D*)_params;
 *
 *         ProcessorBasePtr prc = new ProcessorOdom2D(params);
 *         prc->setName(_name); // pass the name to the created ProcessorOdom2D.
 *         return prc;
 *     }
 *     \endcode
 *
 * #### Achieving automatic registration
 * Currently, registering is performed in each specific ProcessorXxxx source file, processor_xxxx.cpp.
 * For example, in processor_odom_2D.cpp we find the line:
 *
 *     \code
 *     const bool registered_odom_2D = ProcessorFactory::get().registerCreator("ODOM 2D", ProcessorOdom2D::create);
 *     \endcode
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the ProcessorOdom2D class).
 * Therefore, at application level, all processors that have a .cpp file compiled are automatically registered.
 *
 * #### Unregister processor creators
 * The method unregisterCreator() unregisters the ProcessorXxx::create() method.
 * It only needs to be passed the string of the processor type.
 *
 *     \code
 *     ProcessorFactory::get().unregisterCreator("ODOM 2D");
 *     \endcode
 *
 * #### Creating processors
 * Prior to invoking the creation of a processor of a particular type,
 * you must register the creator for this type into the factory.
 *
 * To create a ProcessorOdom2D, you type:
 *
 *     \code
 *     ProcessorFactory::get().create("ODOM 2D", "main odometry", params_ptr);
 *     \endcode
 *
 * #### Example 1 : using the Factories alone
 * We provide the necessary steps to create a processor of class ProcessorOdom2D in our application,
 * and bind it to a SensorOdom2D:
 *
 *     \code
 *     #include "sensor_odom_2D.h"      // provides SensorOdom2D    and SensorFactory
 *     #include "processor_odom_2D.h"   // provides ProcessorOdom2D and ProcessorFactory
 *
 *     // Note: SensorOdom2D::create()    is already registered, automatically.
 *     // Note: ProcessorOdom2D::create() is already registered, automatically.
 *
 *     // First create the sensor (See SensorFactory for details)
 *     SensorBasePtr sensor_ptr = SensorFactory::get().create ( "ODOM 2D" , "Main odometer" , extrinsics , &intrinsics );
 *
 *     // To create a odometry integrator, provide a type="ODOM 2D", a name="main odometry", and a pointer to the parameters struct:
 *
 *     ProcessorParamsOdom2D  params({...});   // fill in the derived struct (note: ProcessorOdom2D actually has no input params)
 *
 *     ProcessorBasePtr processor_ptr =
 *         ProcessorFactory::get().create ( "ODOM 2D" , "main odometry" , &params );
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
 *     #include "sensor_odom_2D.h"
 *     #include "processor_odom_2D.h"
 *     #include "problem.h"
 *
 *     Problem problem(FRM_PO_2D);
 *     problem.installSensor    ( "ODOM 2D" , "Main odometer" , extrinsics      , &intrinsics );
 *     problem.installProcessor ( "ODOM 2D" , "Odometry"      , "Main odometer" , &params     );
 *     \endcode
 *
 * You can also check the code in the example file ````src/examples/test_wolf_factories.cpp````.
 */
    class WolfVisualizer;
    typedef Factory<WolfVisualizer> VisualizerFactory;
template<>
inline std::string VisualizerFactory::getClass()
{
  return "VisualizerFactory";
}


#define WOLF_REGISTER_VISUALIZER(VisualizerType)                        \
    namespace{ const bool WOLF_UNUSED VisualizerType##Registered =      \
            wolf::VisualizerFactory::get().registerCreator(#VisualizerType, VisualizerType::create); } \

} /* namespace wolf */
#endif /* VISUALIZER_FACTORY_H_ */
