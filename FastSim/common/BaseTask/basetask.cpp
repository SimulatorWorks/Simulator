#include "basetask.h"

#include <string>
#include <boost/date_time/posix_time/posix_time_types.hpp>


namespace task{
/**
 * @brief define base class of task
 */


BaseTask::BaseTask()
{
public:

    bool _initialize( void );
    bool _run( void );
    void _cleanup( void );

    virtual bool initialize( void ) = 0;
    virtual bool run( void ) = 0;
    virtual bool cleanup( void ) = 0;

    const std::string& getTaskName( void ) const;
    static boost::posix_time::ptime getSystemTime( void );

protected:

    virtual ~BaseTask;



}; // end BaskTask class


} // end namespace task
