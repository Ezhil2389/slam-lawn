import React, { useState, useEffect } from 'react';
import { Wifi, WifiOff, Camera } from 'lucide-react';
import RobotDashboard from './RobotDashboard';

export const RobotConnection = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [ipAddress, setIpAddress] = useState(() => {
    // Initialize IP from localStorage if available
    const savedIP = localStorage.getItem('robotIP');
    return savedIP || '';
  });
  const [cameraStatus, setCameraStatus] = useState('unknown');
  const [statusCheckInterval, setStatusCheckInterval] = useState(null);

  const checkStatus = async () => {
    try {
      const response = await fetch(`http://${ipAddress}:8000/status`, {
        method: 'GET',
        headers: {
          'Accept': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();
        setIsConnected(true);
        setConnectionStatus('Connected');
        setCameraStatus(data.camera);
      } else {
        throw new Error('Connection lost');
      }
    } catch (error) {
      setIsConnected(false);
      setConnectionStatus('Connection lost');
      setCameraStatus('unknown');
      clearInterval(statusCheckInterval);
      localStorage.removeItem('robotIP');
      console.error('Status check error:', error);
    }
  };

  const handleConnect = async () => {
    if (!ipAddress) {
      setConnectionStatus('Please enter IP address');
      return;
    }

    setConnectionStatus('Connecting...');
    
    try {
      const response = await fetch(`http://${ipAddress}:8000/status`);
      
      if (response.ok) {
        const data = await response.json();
        setIsConnected(true);
        setConnectionStatus('Connected');
        setCameraStatus(data.camera);
        localStorage.setItem('robotIP', ipAddress);
        
        // Start periodic status checking
        const intervalId = setInterval(checkStatus, 5000);
        setStatusCheckInterval(intervalId);
      } else {
        throw new Error('Connection failed');
      }
    } catch (error) {
      setConnectionStatus('Connection failed');
      setIsConnected(false);
      setCameraStatus('unknown');
      localStorage.removeItem('robotIP');
      console.error('Connection error:', error);
    }
  };

  const handleDisconnect = () => {
    setIsConnected(false);
    setConnectionStatus('Disconnected');
    setCameraStatus('unknown');
    clearInterval(statusCheckInterval);
    localStorage.removeItem('robotIP');
  };

  useEffect(() => {
    // Cleanup interval on component unmount
    return () => {
      if (statusCheckInterval) {
        clearInterval(statusCheckInterval);
      }
    };
  }, [statusCheckInterval]);

  if (isConnected) {
    return <RobotDashboard ipAddress={ipAddress} />;
  }
  
  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-green-50 pt-24">
      <div className="container mx-auto px-6">
        <div className="max-w-2xl mx-auto bg-white rounded-3xl shadow-xl p-8">
          <div className="flex items-center justify-between mb-8">
            <h2 className="text-3xl font-bold text-gray-800">Robot Connection</h2>
            {isConnected ? 
              <Wifi className="text-green-500 w-8 h-8" /> : 
              <WifiOff className="text-red-500 w-8 h-8" />
            }
          </div>
          
          <div className="space-y-6">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Robot IP Address
              </label>
              <input
                type="text"
                value={ipAddress}
                onChange={(e) => setIpAddress(e.target.value)}
                placeholder="Enter robot IP address (e.g., 192.168.1.100)"
                className="w-full px-4 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent"
              />
            </div>
            
            <div className="flex items-center space-x-4 mb-4">
              <div className="flex items-center space-x-2">
                <Camera className={`w-5 h-5 ${cameraStatus === 'connected' ? 'text-green-500' : 'text-red-500'}`} />
                <span className="text-sm font-medium text-gray-600">
                  Camera: {cameraStatus}
                </span>
              </div>
            </div>
            
            <div className="flex items-center justify-between">
              <span className="text-sm font-medium text-gray-600">
                Status: {connectionStatus}
              </span>
              <button
                onClick={isConnected ? handleDisconnect : handleConnect}
                className={`px-6 py-2 rounded-lg font-medium transition-colors duration-300 ${
                  isConnected
                    ? 'bg-red-500 hover:bg-red-600 text-white'
                    : 'bg-blue-600 hover:bg-blue-700 text-white'
                }`}
              >
                {isConnected ? 'Disconnect' : 'Connect'}
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RobotConnection;