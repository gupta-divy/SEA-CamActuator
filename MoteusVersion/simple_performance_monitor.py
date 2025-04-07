import time
import statistics

class SimplePerformanceMonitor:
    """
    Simple performance monitor that tracks loop timing with minimal overhead.
    """
    def __init__(self):
        self.loop_times = []
        self.target_period = None
        self.missed_deadlines = 0
        self.total_cycles = 0
        self.last_timestamp = None
        
    def start(self, target_period):
        """Initialize with target loop period in seconds."""
        self.target_period = target_period
        self.last_timestamp = time.perf_counter()
        return self.last_timestamp
        
    def record_cycle(self):
        """Record a cycle completion and return loop time."""
        current_time = time.perf_counter()
        loop_time = current_time - self.last_timestamp
        
        self.loop_times.append(loop_time)
        self.total_cycles += 1
        
        if loop_time > self.target_period:
            self.missed_deadlines += 1
            
        self.last_timestamp = current_time
        return loop_time
        
    def get_stats(self):
        """Return basic performance statistics."""
        if not self.loop_times:
            return "No data collected"
            
        avg_time = statistics.mean(self.loop_times)
        min_time = min(self.loop_times)
        max_time = max(self.loop_times)
        std_dev = statistics.stdev(self.loop_times) if len(self.loop_times) > 1 else 0
        missed_pct = (self.missed_deadlines / self.total_cycles) * 100 if self.total_cycles > 0 else 0
        
        return {
            "avg_ms": avg_time * 1000,
            "min_ms": min_time * 1000,
            "max_ms": max_time * 1000,
            "std_dev_ms": std_dev * 1000,
            "missed_deadlines": self.missed_deadlines,
            "total_cycles": self.total_cycles,
            "missed_pct": missed_pct
        }
        
    def print_stats(self):
        """Print basic performance statistics."""
        stats = self.get_stats()
        if isinstance(stats, str):
            print(stats)
            return
            
        print(f"\nPerformance Statistics:")
        print(f"Average loop time: {stats['avg_ms']:.3f}ms (target: {self.target_period*1000:.3f}ms)")
        print(f"Min/Max loop time: {stats['min_ms']:.3f}ms / {stats['max_ms']:.3f}ms")
        print(f"Loop time standard deviation: {stats['std_dev_ms']:.3f}ms")
        print(f"Missed deadlines: {stats['missed_deadlines']}/{stats['total_cycles']} ({stats['missed_pct']:.1f}%)")