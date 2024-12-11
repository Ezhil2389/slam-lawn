import React, { useState, useEffect } from 'react';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Camera, Battery, Wifi, Gauge, Navigation } from 'lucide-react';
import MapComponent from './MapComponent';
import ScheduleTimer from './ScheduleTimer';

export const RobotDashboard = ({ ipAddress, protocol }) => {
  const [batteryLevel, setBatteryLevel] = useState(100);
  const [speed, setSpeed] = useState(0);
  const [isStreaming, setIsStreaming] = useState(false);
  const [geofenceCoordinates, setGeofenceCoordinates] = useState(null);
  const [isObstacleMode, setIsObstacleMode] = useState(false);

  const handleGeofenceSet = (coordinates) => {
    setGeofenceCoordinates(coordinates);
  };
  
  const handleUseGeofence = () => {
    // This is where you would send the coordinates to the backend
    console.log('Sending geofence coordinates:', geofenceCoordinates);
  };

  const getApiUrl = (endpoint) => {
    const baseProtocol = protocol === 'https:' ? 'https:' : 'http:';
    return `${baseProtocol}//${ipAddress}:8000/${endpoint}`;
  };

  useEffect(() => {
    const handleKeyPress = (e) => {
      switch(e.key) {
        case 'ArrowUp':
          sendCommand('forward');
          break;
        case 'ArrowDown':
          sendCommand('backward');
          break;
        case 'ArrowLeft':
          sendCommand('left');
          break;
        case 'ArrowRight':
          sendCommand('right');
          break;
        case ' ': // Space bar
          sendCommand('stop');
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', handleKeyPress);
    return () => window.removeEventListener('keydown', handleKeyPress);
  }, []);

  const sendCommand = async (command) => {
    try {
      const response = await fetch(getApiUrl('control'), {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ command })
      });
      
      if (!response.ok) {
        console.error('Command failed');
      }
    } catch (error) {
      console.error('Error sending command:', error);
    }
  };

  const toggleStream = () => {
    setIsStreaming(!isStreaming);
  };

  const toggleObstacleMode = async () => {
    try {
      const response = await fetch(getApiUrl('obstacle-mode'), {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ enabled: !isObstacleMode })
      });
      
      if (!response.ok) {
        throw new Error('Failed to toggle obstacle mode');
      }
      
      setIsObstacleMode(!isObstacleMode);
    } catch (error) {
      console.error('Error toggling obstacle mode:', error);
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-green-50">
      <div className="container mx-auto px-6 py-20">
        {protocol === 'https:' && (
          <div className="mb-6 max-w-4xl mx-auto">
            <div className="p-4 bg-yellow-50 rounded-lg border border-yellow-200">
              <p className="text-yellow-800 text-sm">
                You're accessing this page via HTTPS. If you experience connection issues, 
                ensure your robot's API supports HTTPS connections, or consider accessing via HTTP.
              </p>
            </div>
          </div>
        )}
        
        <div className="grid md:grid-cols-3 gap-8">
          {/* Video Feed */}
          <div className="md:col-span-2">
            <div className="bg-white rounded-3xl shadow-xl p-6">
              <div className="flex items-center justify-between mb-4">
                <h2 className="text-2xl font-bold text-gray-800">Live Camera Feed</h2>
                <button
                  onClick={toggleStream}
                  className={`flex items-center px-4 py-2 rounded-lg ${
                    isStreaming ? 'bg-red-500 hover:bg-red-600' : 'bg-blue-600 hover:bg-blue-700'
                  } text-white transition-colors duration-300`}
                >
                  <Camera className="w-5 h-5 mr-2" />
                  {isStreaming ? 'Stop Stream' : 'Start Stream'}
                </button>
              </div>
              <div className="aspect-video bg-gray-900 rounded-xl overflow-hidden">
                {isStreaming ? (
                  <img
                    src={getApiUrl('stream')}
                    alt="Robot camera feed"
                    className="w-full h-full object-cover"
                  />
                ) : (
                  <div className="w-full h-full flex items-center justify-center text-gray-500">
                    <Camera className="w-12 h-12 mb-2" />
                  </div>
                )}
              </div>
            </div>
          </div>

          

          {/* Rest of the dashboard code remains the same */}
          <div className="space-y-6">
            <div className="bg-white rounded-3xl shadow-xl p-6">
              <h3 className="text-xl font-bold text-gray-800 mb-4">Robot Stats</h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between">
                  <span className="text-gray-600">Battery</span>
                  <div className="flex items-center">
                    <Battery className="w-5 h-5 mr-2 text-green-500" />
                    <span className="font-medium">{batteryLevel}%</span>
                  </div>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-gray-600">Speed</span>
                  <div className="flex items-center">
                    <Gauge className="w-5 h-5 mr-2 text-blue-500" />
                    <span className="font-medium">{speed} m/s</span>
                  </div>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-gray-600">Connection</span>
                  <div className="flex items-center">
                    <Wifi className="w-5 h-5 mr-2 text-green-500" />
                    <span className="font-medium">Connected</span>
                  </div>
                </div>
              </div>
            </div>

            <div className="bg-white rounded-3xl shadow-xl p-6">
              <h3 className="text-xl font-bold text-gray-800 mb-4">Manual Controls</h3>
              <div className="grid grid-cols-3 gap-4">
                <div></div>
                <button
                  onClick={() => sendCommand('forward')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-colors duration-300"
                >
                  <ArrowUp className="w-8 h-8 text-blue-600 mx-auto" />
                </button>
                <div></div>
                <button
                  onClick={() => sendCommand('left')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-colors duration-300"
                >
                  <ArrowLeft className="w-8 h-8 text-blue-600 mx-auto" />
                </button>
                <button
                  onClick={() => sendCommand('stop')}
                  className="bg-red-100 hover:bg-red-200 p-4 rounded-xl transition-colors duration-300"
                >
                  <div className="w-4 h-4 bg-red-600 rounded-full mx-auto"></div>
                </button>
                <button
                  onClick={() => sendCommand('right')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-colors duration-300"
                >
                  <ArrowRight className="w-8 h-8 text-blue-600 mx-auto" />
                </button>
                <div></div>
                <button
                  onClick={() => sendCommand('backward')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-colors duration-300"
                >
                  <ArrowDown className="w-8 h-8 text-blue-600 mx-auto" />
                </button>
                <div></div>
              </div>
              
            </div>
            
            <div className="mt-6 flex justify-center">
              <button
                onClick={toggleObstacleMode}
                className={`px-6 py-3 rounded-xl font-medium transition-colors duration-300 flex items-center ${
                  isObstacleMode
                    ? 'bg-red-500 hover:bg-red-600 text-white'
                    : 'bg-green-500 hover:bg-green-600 text-white'
                }`}
              >
                <Navigation className="w-5 h-5 mr-2" />
                {isObstacleMode ? 'Stop Autonomous Mode' : 'Start Autonomous Mode'}
              </button>
            </div>
          </div>
          
        </div>
        <div className="md:col-span-2 mt-8">
  <div className="bg-white rounded-3xl shadow-xl p-6">
    <div className="flex items-center justify-between mb-4">
      <h2 className="text-2xl font-bold text-gray-800">Geofence Setup</h2>
      <button
        onClick={handleUseGeofence}
        disabled={!geofenceCoordinates}
        className={`px-4 py-2 rounded-lg ${
          geofenceCoordinates
            ? 'bg-blue-600 hover:bg-blue-700'
            : 'bg-gray-300 cursor-not-allowed'
        } text-white transition-colors duration-300`}
      >
        Use Geofence
      </button>
    </div>
    <MapComponent onGeofenceSet={handleGeofenceSet} />
    {geofenceCoordinates && (
      <div className="mt-4 p-4 bg-gray-50 rounded-lg">
        <h3 className="text-sm font-medium text-gray-600 mb-2">Geofence Boundary Points:</h3>
        <div className="max-h-40 overflow-y-auto">
          <pre className="text-xs">
            {JSON.stringify(geofenceCoordinates.coordinates, null, 2)}
          </pre>
        </div>
        <p className="text-xs text-gray-500 mt-2">
          {geofenceCoordinates.coordinates.length} points defined
        </p>
      </div>
    )}
  </div>
  <div className="mt-8">
          <ScheduleTimer />
        </div>
</div>
        
      </div>
    </div>
  );
};

export default RobotDashboard;