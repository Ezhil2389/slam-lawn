import React, { useState } from 'react';
import { Clock, Plus, Trash2, Calendar } from 'lucide-react';

const ScheduleTimer = () => {
  const [schedules, setSchedules] = useState([]);
  const [newSchedule, setNewSchedule] = useState({
    days: [],
    startTime: '09:00',
    duration: 60,
  });

  const daysOfWeek = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];

  const handleDayToggle = (day) => {
    setNewSchedule(prev => ({
      ...prev,
      days: prev.days.includes(day)
        ? prev.days.filter(d => d !== day)
        : [...prev.days, day]
    }));
  };

  const handleAddSchedule = () => {
    if (newSchedule.days.length === 0) {
      alert('Please select at least one day');
      return;
    }
    setSchedules([...schedules, { ...newSchedule, id: Date.now() }]);
    setNewSchedule({
      days: [],
      startTime: '09:00',
      duration: 60,
    });
  };

  const handleRemoveSchedule = (id) => {
    setSchedules(schedules.filter(schedule => schedule.id !== id));
  };

  return (
    <div className="bg-white rounded-3xl shadow-xl p-6">
      <div className="flex items-center justify-between mb-6">
        <h2 className="text-2xl font-bold text-gray-800">Mowing Schedule</h2>
        <Calendar className="text-blue-600 w-6 h-6" />
      </div>

      {/* Add new schedule */}
      <div className="bg-gray-50 rounded-xl p-4 mb-6">
        <h3 className="text-lg font-semibold text-gray-700 mb-4">Add New Schedule</h3>
        
        {/* Days selection */}
        <div className="flex flex-wrap gap-2 mb-4">
          {daysOfWeek.map(day => (
            <button
              key={day}
              onClick={() => handleDayToggle(day)}
              className={`px-3 py-1 rounded-full text-sm font-medium transition-colors duration-200 ${
                newSchedule.days.includes(day)
                  ? 'bg-blue-600 text-white'
                  : 'bg-gray-200 text-gray-600 hover:bg-gray-300'
              }`}
            >
              {day}
            </button>
          ))}
        </div>

        {/* Time and duration */}
        <div className="grid grid-cols-2 gap-4 mb-4">
          <div>
            <label className="block text-sm font-medium text-gray-600 mb-1">Start Time</label>
            <input
              type="time"
              value={newSchedule.startTime}
              onChange={(e) => setNewSchedule(prev => ({ ...prev, startTime: e.target.value }))}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-600 mb-1">Duration (minutes)</label>
            <input
              type="number"
              min="15"
              max="240"
              value={newSchedule.duration}
              onChange={(e) => setNewSchedule(prev => ({ ...prev, duration: parseInt(e.target.value) }))}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            />
          </div>
        </div>

        <button
          onClick={handleAddSchedule}
          className="w-full bg-blue-600 text-white px-4 py-2 rounded-lg hover:bg-blue-700 transition-colors duration-300 flex items-center justify-center"
        >
          <Plus className="w-5 h-5 mr-2" />
          Add Schedule
        </button>
      </div>

      {/* Schedule list */}
      <div className="space-y-4">
        {schedules.map(schedule => (
          <div key={schedule.id} className="flex items-center justify-between bg-gray-50 p-4 rounded-xl">
            <div>
              <div className="flex items-center mb-2">
                <Clock className="w-5 h-5 text-blue-600 mr-2" />
                <span className="font-medium">{schedule.startTime}</span>
                <span className="mx-2 text-gray-500">•</span>
                <span className="text-gray-600">{schedule.duration} mins</span>
              </div>
              <div className="flex flex-wrap gap-1">
                {schedule.days.map(day => (
                  <span key={day} className="text-xs font-medium text-gray-600 bg-gray-200 px-2 py-1 rounded-full">
                    {day}
                  </span>
                ))}
              </div>
            </div>
            <button
              onClick={() => handleRemoveSchedule(schedule.id)}
              className="text-red-500 hover:text-red-600 p-2"
            >
              <Trash2 className="w-5 h-5" />
            </button>
          </div>
        ))}
        {schedules.length === 0 && (
          <div className="text-center text-gray-500 py-6">
            No schedules set
          </div>
        )}
      </div>
    </div>
  );
};

export default ScheduleTimer; 