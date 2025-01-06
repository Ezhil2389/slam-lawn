import React, { useState, useEffect } from 'react';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Camera, Battery, Wifi, Gauge, Navigation, StopCircle, GamepadIcon} from 'lucide-react';
import ScheduleTimer from './ScheduleTimer';
import LocalizationStats from './LocalizationStats';

export const RobotDashboard = ({ ipAddress, protocol, isConnected, onDisconnect }) => {
  const [isStreaming, setIsStreaming] = useState(false);
  const [geofenceLength, setGeofenceLength] = useState('');
  const [geofenceBreadth, setGeofenceBreadth] = useState('');
  const [isObstacleMode, setIsObstacleMode] = useState(false);

  const handleGeofenceSubmit = async () => {
    try {
      const response = await fetch(getApiUrl('geofence'), {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          length: parseFloat(geofenceLength),
          breadth: parseFloat(geofenceBreadth)
        })
      });
      
      if (!response.ok) {
        throw new Error('Failed to set geofence');
      }
      
      alert('Geofence set successfully');
    } catch (error) {
      console.error('Error setting geofence:', error);
      alert('Failed to set geofence');
    }
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

          {/* Controls Section */}
          <div className="space-y-6">
            <div className="bg-white rounded-3xl shadow-xl p-8 mb-8">
              <div className="flex flex-col items-center justify-center space-y-4">
                <div className={`w-full flex items-center justify-between space-x-4 px-8 py-6 rounded-2xl ${
                  isConnected 
                    ? 'bg-green-50 border-2 border-green-200'
                    : 'bg-red-50 border-2 border-red-200'
                }`}>
                  <div className="flex items-center space-x-4">
                    <Wifi className={`w-8 h-8 ${
                      isConnected ? 'text-green-600' : 'text-red-600'
                    }`} />
                    <div className="flex flex-col">
                      <span className="text-xl font-semibold">
                        {isConnected ? 'Connected' : 'Disconnected'}
                      </span>
                      {isConnected && (
                        <span className="text-sm text-green-600">
                          {ipAddress}
                        </span>
                      )}
                    </div>
                  </div>
                  {isConnected && (
                    <button
                      onClick={onDisconnect}
                      className="px-4 py-2 bg-red-500 hover:bg-red-600 text-white rounded-lg transition-colors duration-300"
                    >
                      Disconnect
                    </button>
                  )}
                </div>
              </div>
            </div>

            <div className="bg-white rounded-3xl shadow-xl p-6 backdrop-blur-sm bg-white/80">
              <h3 className="text-xl font-bold text-gray-800 mb-6 flex items-center">
                <GamepadIcon className="w-6 h-6 mr-2 text-blue-600" />
                Manual Controls
              </h3>
              
              <div className="grid grid-cols-3 gap-4 max-w-md mx-auto">
                <div></div>
                <button
                  onClick={() => sendCommand('forward')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-all duration-300 hover:shadow-md active:scale-95"
                >
                  <ArrowUp className="w-8 h-8 text-blue-600 mx-auto" />
                </button>
                <div></div>
                
                <button
                  onClick={() => sendCommand('left')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-all duration-300 hover:shadow-md active:scale-95"
                >
                  <ArrowLeft className="w-8 h-8 text-blue-600 mx-auto" />
                </button>
                <button
                  onClick={() => sendCommand('stop')}
                  className="bg-red-100 hover:bg-red-200 p-4 rounded-xl transition-all duration-300 hover:shadow-md active:scale-95"
                >
                  <StopCircle className="w-8 h-8 text-red-600 mx-auto" />
                </button>
                <button
                  onClick={() => sendCommand('right')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-all duration-300 hover:shadow-md active:scale-95"
                >
                  <ArrowRight className="w-8 h-8 text-blue-600 mx-auto" />
                </button>

                <div></div>
                <button
                  onClick={() => sendCommand('backward')}
                  className="bg-blue-100 hover:bg-blue-200 p-4 rounded-xl transition-all duration-300 hover:shadow-md active:scale-95"
                >
                  <ArrowDown className="w-8 h-8 text-blue-600 mx-auto" />
                </button>
                <div></div>
              </div>
            </div>
            
            <div className="mt-8">
              <button
                onClick={toggleObstacleMode}
                className={`
                  w-full max-w-2xl mx-auto
                  px-8 py-6 rounded-2xl
                  font-medium
                  transition-all duration-300
                  flex items-center justify-center space-x-4
                  shadow-lg hover:shadow-xl
                  ${
                    isObstacleMode
                      ? 'bg-red-500 hover:bg-red-600 text-white'
                      : 'bg-green-500 hover:bg-green-600 text-white'
                  }
                `}
              >
                <Navigation className="w-8 h-8" />
                <span className="text-xl font-semibold">
                  {isObstacleMode ? 'Stop Autonomous Mode' : 'Start Autonomous Mode'}
                </span>
              </button>
            </div>
          </div>
        </div>

        {/* Geofence and Schedule Section */}
        <div className="md:col-span-2 mt-8">
          <div className="bg-white rounded-3xl shadow-xl p-6">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-2xl font-bold text-gray-800">Geofence Setup</h2>
            </div>
            <div className="grid grid-cols-2 gap-4 mb-4">
              <div>
                <label className="block text-sm font-medium text-gray-600 mb-2">
                  Length (metres)
                </label>
                <input
                  type="number"
                  min="1"
                  value={geofenceLength}
                  onChange={(e) => setGeofenceLength(e.target.value)}
                  className="w-full px-4 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-600 mb-2">
                  Breadth (metres)
                </label>
                <input
                  type="number"
                  min="1"
                  value={geofenceBreadth}
                  onChange={(e) => setGeofenceBreadth(e.target.value)}
                  className="w-full px-4 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                />
              </div>
            </div>
            <button
              onClick={handleGeofenceSubmit}
              disabled={!geofenceLength || !geofenceBreadth}
              className={`w-full px-4 py-2 rounded-lg ${
                geofenceLength && geofenceBreadth
                  ? 'bg-blue-600 hover:bg-blue-700'
                  : 'bg-gray-300 cursor-not-allowed'
              } text-white transition-colors duration-300`}
            >
              Set Geofence
            </button>
          </div>
          <div className="mt-8">
            <ScheduleTimer />
          </div>
        </div>

        {/* Add LocalizationStats at the bottom */}
        <div className="mt-8">
          <LocalizationStats 
            isAutonomous={isObstacleMode}
          />
        </div>
      </div>
    </div>
  );
};

export default RobotDashboard;