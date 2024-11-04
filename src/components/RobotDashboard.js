import React, { useState, useEffect } from 'react';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Camera, Battery, Wifi, Gauge } from 'lucide-react';

export const RobotDashboard = ({ ipAddress }) => {
  const [videoStream, setVideoStream] = useState(null);
  const [batteryLevel, setBatteryLevel] = useState(100);
  const [speed, setSpeed] = useState(0);
  const [isStreaming, setIsStreaming] = useState(false);

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
      const response = await fetch(`http://${ipAddress}:8000/control`, {
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

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-green-50 pt-24">
      <div className="container mx-auto px-6">
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
                    src={`http://${ipAddress}:8000/stream`}
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

          {/* Controls and Stats */}
          <div className="space-y-6">
            {/* Stats */}
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

            {/* Controls */}
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
          </div>
        </div>
      </div>
    </div>
  );
};

export default RobotDashboard;