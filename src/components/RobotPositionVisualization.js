import React, { useEffect, useRef, useState } from 'react';

const RobotPositionVisualization = ({ ipAddress, protocol }) => {
  const canvasRef = useRef(null);
  const [dimensions, setDimensions] = useState({ length: 0, breadth: 0 });

  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    let animationFrameId;

    const drawVisualization = (x, y, areaLength, areaBreadth) => {
      // Clear the canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Set dimensions if they've changed
      if (areaLength !== dimensions.length || areaBreadth !== dimensions.breadth) {
        setDimensions({ length: areaLength, breadth: areaBreadth });
      }

      // Draw the boundary box
      ctx.strokeStyle = '#2563eb';
      ctx.lineWidth = 2;
      ctx.strokeRect(0, 0, canvas.width, canvas.height);

      // Add measurements
      ctx.font = '14px Arial';
      ctx.fillStyle = '#4b5563';
      ctx.textAlign = 'center';
      // Length label (top)
      ctx.fillText(`${areaLength}m`, canvas.width / 2, 20);
      // Breadth label (side)
      ctx.save();
      ctx.translate(20, canvas.height / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText(`${areaBreadth}m`, 0, 0);
      ctx.restore();

      // Draw grid
      ctx.strokeStyle = '#e5e7eb';
      ctx.lineWidth = 0.5;
      
      // Draw grid lines every meter
      const pixelsPerMeterX = canvas.width / areaLength;
      const pixelsPerMeterY = canvas.height / areaBreadth;

      // Vertical lines
      for (let i = 1; i < areaLength; i++) {
        const x = i * pixelsPerMeterX;
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
      }

      // Horizontal lines
      for (let i = 1; i < areaBreadth; i++) {
        const y = i * pixelsPerMeterY;
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }

      // Calculate robot position in canvas coordinates
      const scaleX = canvas.width / areaLength;
      const scaleY = canvas.height / areaBreadth;
      const canvasX = (x + areaLength/2) * scaleX;
      const canvasY = canvas.height - ((y + areaBreadth/2) * scaleY);

      // Draw robot position
      ctx.beginPath();
      ctx.fillStyle = '#ef4444'; // Red dot for robot
      ctx.arc(canvasX, canvasY, 8, 0, Math.PI * 2);
      ctx.fill();

      // Add position text
      ctx.fillStyle = '#000000';
      ctx.font = '12px Arial';
      ctx.textAlign = 'center';
      ctx.fillText(`(${x.toFixed(2)}m, ${y.toFixed(2)}m)`, canvasX, canvasY - 15);
    };

    const updatePosition = async () => {
      try {
        const response = await fetch(`${protocol}//${ipAddress}:8000/position`);
        if (!response.ok) throw new Error('Failed to fetch position');
        const data = await response.json();
        drawVisualization(data.x, data.y, data.areaLength, data.areaBreadth);
      } catch (error) {
        console.error('Error fetching robot position:', error);
      }
      animationFrameId = requestAnimationFrame(updatePosition);
    };

    updatePosition();

    return () => {
      cancelAnimationFrame(animationFrameId);
    };
  }, [ipAddress, protocol, dimensions]);

  return (
    <div className="bg-white rounded-3xl shadow-xl p-6">
      <h3 className="text-xl font-bold text-gray-800 mb-4">Robot Position Monitor</h3>
      <div className="relative">
        <canvas
          ref={canvasRef}
          width={400}
          height={400}
          className="w-full border border-gray-200 rounded-xl"
        />
      </div>
    </div>
  );
};

export default RobotPositionVisualization; 