#!/bin/bash

# Emergency script to disable gravity compensation on all controllers
# Use this if something goes wrong during testing

echo "🚨 EMERGENCY: Disabling gravity compensation on all controllers..."

# List of controllers to disable
controllers=(
    "/sciurus17/controller1/joints/gravity_compensation"
    "/sciurus17/controller2/joints/gravity_compensation"
    "/sciurus17/controller3/joints/gravity_compensation"
)

# Disable gravity compensation for each controller
for controller in "${controllers[@]}"; do
    echo "Disabling $controller..."
    rosservice call "$controller/set_parameters" "{config: {enable: false}}" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "✅ Successfully disabled $controller"
    else
        echo "❌ Failed to disable $controller (service may not exist)"
    fi
done

echo ""
echo "🔍 Verification - checking current status:"
python3 -c "
import os
os.system('rosservice list | grep gravity | head -3')
print('\\nIf you see services listed above, gravity compensation is available.')
print('Run the test script to verify current status:')
print('   python3 test_gravity_compensation.py')
"

echo ""
echo "✅ Emergency disable complete!"
echo "💡 To verify status, run: python3 test_gravity_compensation.py"