std_msgs/Header header
# 
string   signal_id              # signal identifaction of event
int16    SEVERITY_WARNING = 0   
int16    SEVERITY_ERROR = 1
int16    SEVERITY_FATAL = 2   
int16    severity               # severity of event
bool     eval                   # evaluated true/false value
float64  val                    # value of signal