#!/usr/bin/env python3
import rospy
import time
from custom.srv import add, addResponse   # Import both service type and response

class AddServer:
    def __init__(self):
        # Initialize statistics and history
        self.request_count = 0
        self.operation_history = []
        self.start_time = time.time()
        self.total_sum = 0
        self.max_result = None
        self.min_result = None
        
        rospy.loginfo("Enhanced Add Server initialized with statistics tracking")
    
    def validate_inputs(self, a, b):
        """Validate input parameters"""
        # Check for potential overflow (Python handles big integers well, but let's set reasonable limits)
        MAX_VALUE = 10**15
        MIN_VALUE = -10**15
        
        if a > MAX_VALUE or a < MIN_VALUE:
            rospy.logwarn("Input 'a' (%d) is outside safe range [%d, %d]", a, MIN_VALUE, MAX_VALUE)
            return False, "Input 'a' is outside safe range"
        
        if b > MAX_VALUE or b < MIN_VALUE:
            rospy.logwarn("Input 'b' (%d) is outside safe range [%d, %d]", b, MIN_VALUE, MAX_VALUE)
            return False, "Input 'b' is outside safe range"
        
        return True, "Valid inputs"
    
    def update_statistics(self, a, b, result):
        """Update server statistics"""
        self.request_count += 1
        self.total_sum += result
        
        # Update min/max results
        if self.max_result is None or result > self.max_result:
            self.max_result = result
        if self.min_result is None or result < self.min_result:
            self.min_result = result
        
        # Store operation in history (keep last 100 operations)
        operation = {
            'timestamp': time.time(),
            'a': a,
            'b': b,
            'result': result,
            'request_id': self.request_count
        }
        self.operation_history.append(operation)
        
        # Keep only last 100 operations to prevent memory issues
        if len(self.operation_history) > 100:
            self.operation_history.pop(0)
    
    def log_statistics(self):
        """Log current server statistics"""
        uptime = time.time() - self.start_time
        avg_result = self.total_sum / self.request_count if self.request_count > 0 else 0
        
        rospy.loginfo("=== ADD SERVER STATISTICS ===")
        rospy.loginfo("Uptime: %.2f seconds", uptime)
        rospy.loginfo("Total requests: %d", self.request_count)
        rospy.loginfo("Average result: %.2f", avg_result)
        rospy.loginfo("Max result: %s", self.max_result)
        rospy.loginfo("Min result: %s", self.min_result)
        rospy.loginfo("Requests per minute: %.2f", (self.request_count / uptime * 60) if uptime > 0 else 0)
        rospy.loginfo("=============================")
    
    def get_operation_summary(self, a, b):
        """Generate a summary of the operation"""
        abs_a, abs_b = abs(a), abs(b)
        
        # Determine operation characteristics
        characteristics = []
        if a == 0 or b == 0:
            characteristics.append("zero-addition")
        if a == b:
            characteristics.append("doubling")
        if a < 0 and b < 0:
            characteristics.append("negative-sum")
        elif a < 0 or b < 0:
            characteristics.append("mixed-signs")
        if abs_a > 1000 or abs_b > 1000:
            characteristics.append("large-numbers")
        
        return characteristics
    
    def add_handler(self, req):
        """Enhanced addition handler with validation and statistics"""
        start_time = time.time()
        
        # Validate inputs
        is_valid, validation_msg = self.validate_inputs(req.a, req.b)
        if not is_valid:
            rospy.logerr("Invalid input: %s", validation_msg)
            # Return 0 for invalid inputs (could also raise an exception)
            return addResponse(0)
        
        # Perform the addition
        result = req.a + req.b
        
        # Get operation characteristics
        characteristics = self.get_operation_summary(req.a, req.b)
        char_str = ", ".join(characteristics) if characteristics else "standard"
        
        # Update statistics
        self.update_statistics(req.a, req.b, result)
        
        # Calculate processing time
        processing_time = (time.time() - start_time) * 1000  # in milliseconds
        
        # Enhanced logging
        rospy.loginfo("Request #%d: %d + %d = %d [%s] (%.2fms)", 
                     self.request_count, req.a, req.b, result, char_str, processing_time)
        
        # Log statistics every 10 requests
        if self.request_count % 10 == 0:
            self.log_statistics()
        
        return addResponse(result)
    
    def shutdown_handler(self):
        """Handle server shutdown gracefully"""
        rospy.loginfo("Add server shutting down...")
        self.log_statistics()
        
        if self.operation_history:
            rospy.loginfo("Last 5 operations:")
            for op in self.operation_history[-5:]:
                rospy.loginfo("  #%d: %d + %d = %d (%.2fs ago)", 
                             op['request_id'], op['a'], op['b'], op['result'],
                             time.time() - op['timestamp'])

def main():
    """Main function to run the enhanced add server"""
    rospy.init_node('add_server', anonymous=True)
    
    # Create server instance
    server = AddServer()
    
    # Register shutdown handler
    rospy.on_shutdown(server.shutdown_handler)
    
    # Create the service
    service = rospy.Service("add", add, server.add_handler)
    rospy.loginfo("Enhanced Add service is ready and listening...")
    rospy.loginfo("Service features: input validation, statistics tracking, operation history")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Add server interrupted by user")

if __name__ == "__main__":
    main()
