#!/usr/bin/env python3

import rospy
import time
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.srv import Reconfigure

def get_available_controllers():
    """
    Discover available gravity compensation controllers
    """
    controllers = []
    try:
        # Get list of services
        import rosservice
        services = rosservice.get_service_list()
        
        for service in services:
            if 'gravity_compensation/set_parameters' in service:
                # Extract controller name from service path
                # /sciurus17/controller1/joints/gravity_compensation/set_parameters
                parts = service.split('/')
                if len(parts) >= 3:
                    controller_path = '/'.join(parts[:-2])  # Remove /gravity_compensation/set_parameters
                    controllers.append(controller_path)
                    
    except Exception as e:
        print(f"⚠️  Could not auto-discover controllers: {e}")
        # Fallback to known controller names
        controllers = [
            "/sciurus17/controller1/joints",
            "/sciurus17/controller2/joints", 
            "/sciurus17/controller3/joints"
        ]
    
    return controllers

def test_single_controller(controller_path):
    """
    Test gravity compensation for a single controller
    """
    reconfigure_path = f"{controller_path}/gravity_compensation"
    print(f"\n🎯 Testing controller: {controller_path}")
    
    try:
        # Connect to the reconfigure server
        client = Client(reconfigure_path, timeout=3.0)
        print(f"✅ Connected to {reconfigure_path}")
        
        # Get current configuration
        current_config = client.get_configuration()
        print(f"📊 Current config: enable={current_config.get('enable', 'N/A')}, gain={current_config.get('gain', 'N/A')}")
        
        # Test 1: Enable debug output
        print("   🧪 Enabling debug output...")
        client.update_configuration({"debug_output": True})
        time.sleep(0.5)
        
        # Test 2: Test enable with very low gain (SAFE)
        print("   🧪 Enabling gravity compensation with gain 0.05...")
        client.update_configuration({"enable": True, "gain": 0.05})
        time.sleep(1.5)
        print("   ✅ Enabled successfully")
        
        # Test 3: Disable for safety
        print("   🧪 Disabling gravity compensation...")
        client.update_configuration({"enable": False})
        time.sleep(0.5)
        
        # Test 4: Reset debug output
        client.update_configuration({"debug_output": False})
        
        print("   ✅ All tests passed for this controller")
        return True
        
    except Exception as e:
        print(f"   ❌ Error testing {controller_path}: {e}")
        return False

def test_gravity_compensation_system():
    """
    Test the complete gravity compensation system
    """
    rospy.init_node('gravity_compensation_tester', anonymous=True)
    
    print("🚀 Gravity Compensation System Test")
    print("=" * 60)
    
    # Discover controllers
    print("🔍 Discovering available controllers...")
    controllers = get_available_controllers()
    
    if not controllers:
        print("❌ No gravity compensation controllers found!")
        return False
    
    print(f"📡 Found {len(controllers)} controllers:")
    for controller in controllers:
        print(f"   - {controller}")
    
    # Test each controller
    success_count = 0
    for controller in controllers:
        if test_single_controller(controller):
            success_count += 1
    
    # Summary
    print(f"\n📊 Test Summary:")
    print(f"   Controllers tested: {len(controllers)}")
    print(f"   Successful: {success_count}")
    print(f"   Failed: {len(controllers) - success_count}")
    
    if success_count == len(controllers):
        print("\n🎉 ALL TESTS PASSED!")
        return True
    else:
        print(f"\n⚠️  {len(controllers) - success_count} controllers failed testing")
        return False

def show_usage_examples():
    """
    Show practical usage examples with actual service paths
    """
    print("\n📚 Usage Examples:")
    print("=" * 50)
    
    print("\n1. Using rqt_reconfigure (GUI - RECOMMENDED):")
    print("   rosrun rqt_reconfigure rqt_reconfigure")
    print("   Navigate to:")
    print("   - /sciurus17/controller1/joints/gravity_compensation")
    print("   - /sciurus17/controller2/joints/gravity_compensation")
    print("   - /sciurus17/controller3/joints/gravity_compensation")
    
    print("\n2. Using dynamic_reconfigure Python API:")
    print("   from dynamic_reconfigure.client import Client")
    print("   client = Client('/sciurus17/controller1/joints/gravity_compensation')")
    print("   client.update_configuration({'enable': True, 'gain': 0.1})")
    
    print("\n3. Using rosservice (Advanced):")
    print("   # Enable gravity compensation")
    print("   rosservice call /sciurus17/controller1/joints/gravity_compensation/set_parameters \\")
    print("   \"{config: {enable: true, gain: 0.1, debug_output: true}}\"")
    
    print("\n4. Safe Testing Sequence:")
    print("   # Start with ONE controller only")
    print("   controller1: enable=true, gain=0.1")
    print("   # Monitor for 30 seconds")
    print("   # If stable, enable others one by one")
    print("   # Gradually increase gain: 0.1 → 0.2 → 0.3")

def show_emergency_commands():
    """
    Show emergency disable commands
    """
    print("\n🚨 EMERGENCY DISABLE COMMANDS:")
    print("=" * 40)
    print("If something goes wrong, run these commands immediately:")
    print()
    
    controllers = [
        "/sciurus17/controller1/joints/gravity_compensation",
        "/sciurus17/controller2/joints/gravity_compensation", 
        "/sciurus17/controller3/joints/gravity_compensation"
    ]
    
    for controller in controllers:
        print(f"rosservice call {controller}/set_parameters \"{{config: {{enable: false}}}}\"")
    
    print("\nOr use Python:")
    print("from dynamic_reconfigure.client import Client")
    for controller in controllers:
        print(f"Client('{controller}').update_configuration({{'enable': False}})")

def interactive_test():
    """
    Interactive testing mode
    """
    print("\n🎮 Interactive Test Mode")
    print("=" * 30)
    
    controllers = get_available_controllers()
    
    while True:
        print(f"\nAvailable controllers:")
        for i, controller in enumerate(controllers):
            print(f"{i+1}. {controller}")
        print("0. Exit")
        
        try:
            choice = input("\nSelect controller to test (0 to exit): ")
            choice = int(choice)
            
            if choice == 0:
                break
            elif 1 <= choice <= len(controllers):
                controller = controllers[choice-1]
                
                print(f"\nTesting {controller}...")
                enable = input("Enable gravity compensation? (y/N): ").lower() == 'y'
                
                if enable:
                    gain = float(input("Enter gain (0.0-2.0, recommended 0.1): ") or "0.1")
                    debug = input("Enable debug output? (y/N): ").lower() == 'y'
                    
                    try:
                        client = Client(f"{controller}/gravity_compensation")
                        client.update_configuration({
                            'enable': True,
                            'gain': gain,
                            'debug_output': debug
                        })
                        print(f"✅ Enabled gravity compensation: gain={gain}, debug={debug}")
                    except Exception as e:
                        print(f"❌ Error: {e}")
                else:
                    try:
                        client = Client(f"{controller}/gravity_compensation")
                        client.update_configuration({'enable': False})
                        print("✅ Disabled gravity compensation")
                    except Exception as e:
                        print(f"❌ Error: {e}")
            else:
                print("Invalid choice!")
                
        except ValueError:
            print("Please enter a number!")
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--interactive":
        rospy.init_node('gravity_compensation_tester', anonymous=True)
        interactive_test()
    else:
        # Run automated tests
        success = test_gravity_compensation_system()
        
        if success:
            show_usage_examples()
            show_emergency_commands()
        else:
            print("\n🔧 Troubleshooting:")
            print("1. Is the robot control node running?")
            print("2. Check for 'Gravity compensation initialized successfully' message")
            print("3. Verify services exist:")
            print("   rosservice list | grep gravity")
            print("4. Try interactive mode:")
            print("   python3 test_gravity_compensation.py --interactive")