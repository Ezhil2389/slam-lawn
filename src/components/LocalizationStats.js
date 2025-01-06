import React, { useEffect, useState } from 'react';
import { MapPin, Compass, Gauge, RotateCw } from 'lucide-react';

const LocalizationStats = ({ isAutonomous }) => {
  const [robotPose, setRobotPose] = useState({
    x: 0,
    y: 0,
    theta: 0,
    distanceTraveled: 0,
    angularVelocity: 0
  });
  const [updateStatus, setUpdateStatus] = useState('idle');

  useEffect(() => {
    const fetchPoseData = async () => {
      try {
        setUpdateStatus('loading');
        const host = window.location.hostname || 'localhost';
        const url = `http://${host}:8000/pose`;
        console.log('Fetching from:', url); // Debug log
        
        const response = await fetch(url, {
          method: 'GET',
          headers: {
            'Accept': 'application/json',
            'Content-Type': 'application/json',
          },
        });
        
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        console.log('Received pose data:', data); // Debug log
        setRobotPose(data);
        setUpdateStatus('success');
      } catch (error) {
        console.error('Error fetching pose:', error);
        setUpdateStatus('error');
      }
    };

    // Fetch immediately on mount
    fetchPoseData();

    // Then fetch every 2 seconds (less frequent to reduce load)
    const interval = setInterval(fetchPoseData, 2000);

    // Cleanup on unmount
    return () => clearInterval(interval);
  }, []); // Empty dependency array - only run once on mount

  // Convert radians to degrees for display
  const headingDegrees = (robotPose?.theta * 180 / Math.PI) || 0;

  return (
    <div className="bg-white rounded-3xl shadow-xl p-6">
      <h3 className="text-xl font-bold text-gray-800 mb-6 flex items-center justify-between">
        <div className="flex items-center">
          <MapPin className="w-6 h-6 mr-2 text-blue-600" />
          Robot Localization
        </div>
        <div className={`text-sm ${
          updateStatus === 'success' ? 'text-green-600' : 
          updateStatus === 'error' ? 'text-red-600' : 
          'text-yellow-600'
        }`}>
          Status: {updateStatus}
        </div>
      </h3>
      
      <div className="grid grid-cols-2 gap-4">
        {/* Position */}
        <div className="bg-blue-50 rounded-xl p-4">
          <div className="text-sm text-blue-600 font-medium mb-1">Position</div>
          <div className="grid grid-cols-2 gap-2">
            <div>
              <span className="text-sm text-gray-500">X:</span>
              <span className="text-lg font-semibold ml-2">
                {robotPose.x.toFixed(2)}m
              </span>
            </div>
            <div>
              <span className="text-sm text-gray-500">Y:</span>
              <span className="text-lg font-semibold ml-2">
                {robotPose.y.toFixed(2)}m
              </span>
            </div>
          </div>
        </div>

        {/* Heading */}
        <div className="bg-green-50 rounded-xl p-4">
          <div className="text-sm text-green-600 font-medium mb-1">Heading</div>
          <div className="flex items-center">
            <Compass className="w-5 h-5 text-green-600 mr-2" />
            <span className="text-lg font-semibold">
              {headingDegrees.toFixed(1)}°
            </span>
          </div>
        </div>

        {/* Distance Traveled */}
        <div className="bg-purple-50 rounded-xl p-4">
          <div className="text-sm text-purple-600 font-medium mb-1">Distance Traveled</div>
          <div className="flex items-center">
            <Gauge className="w-5 h-5 text-purple-600 mr-2" />
            <span className="text-lg font-semibold">
              {robotPose.distanceTraveled.toFixed(2)}m
            </span>
          </div>
        </div>

        {/* Angular Velocity */}
        <div className="bg-orange-50 rounded-xl p-4">
          <div className="text-sm text-orange-600 font-medium mb-1">Angular Velocity</div>
          <div className="flex items-center">
            <RotateCw className="w-5 h-5 text-orange-600 mr-2" />
            <span className="text-lg font-semibold">
              {robotPose.angularVelocity.toFixed(2)}°/s
            </span>
          </div>
        </div>
      </div>

      <div className="mt-4 p-3 bg-gray-50 rounded-lg">
        <div className="text-sm text-gray-500">
          Status: {isAutonomous ? (
            <span className="text-green-600 font-medium">Autonomous Navigation Active</span>
          ) : (
            <span className="text-gray-600 font-medium">Manual Control</span>
          )}
        </div>
      </div>
    </div>
  );
};

export default LocalizationStats; 