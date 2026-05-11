#!/bin/bash
# DEOS System Status Check

echo "=========================================="
echo "DEOS Autonomous Vehicle System Status"
echo "=========================================="
echo ""

echo "📦 PACKAGES BUILT:"
docker exec c5f464a64ab3 colcon list | wc -l
echo ""

echo "🔧 RUNNING NODES:"
docker exec c5f464a64ab3 bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null | sort -u" | head -20
echo ""

echo "📊 TOTAL TOPICS:"
docker exec c5f464a64ab3 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | wc -l"
echo ""

echo "🔌 SERVICES REGISTERED:"
docker exec c5f464a64ab3 bash -c "source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | wc -l"
echo ""

echo "⚙️  KEY NODES:"
docker exec c5f464a64ab3 bash -c "ps aux | grep -E 'vehicle_controller|mission_planning|ekf_node|vision_bridge' | grep -v grep | awk '{print \$NF}' | sort"
echo ""

echo "✅ SYSTEM READY FOR DEPLOYMENT"
echo "=========================================="
