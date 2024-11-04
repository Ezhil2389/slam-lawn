import React, { useState, useEffect, useCallback, useRef } from 'react';
import { MapPin, Navigation, Play, Pause, TreeDeciduous, Info, Battery, Zap, ChevronDown, Menu, X, RotateCcw, Wifi } from 'lucide-react';
import { RobotConnection } from './components/RobotConnection';
import { RobotDashboard } from './components/RobotDashboard';
import './index.css';

const Section = ({ id, title, children, bgColor = 'bg-white' }) => (
  <section id={id} className={`min-h-screen ${bgColor} py-20 relative overflow-hidden`}>
    <div className="container mx-auto px-6 relative z-10">
      <h2 className="text-5xl font-extrabold mb-12 text-gray-800 tracking-tight">{title}</h2>
      {children}
    </div>
    <div className="absolute inset-0 bg-gradient-to-br from-transparent to-black opacity-5"></div>
  </section>
);

const NavLink = ({ href, children, isActive, onClick }) => (
  <a
    href={href}
    onClick={onClick}
    className={`text-lg font-medium transition-all duration-300 block ${
      isActive 
        ? 'text-blue-600 bg-blue-100 px-4 py-2 rounded-full' 
        : 'text-gray-600 hover:text-blue-600 hover:bg-blue-50 px-4 py-2 rounded-full'
    }`}
  >
    {children}
  </a>
);

const FeatureCard = ({ icon: Icon, title, description }) => (
  <div className="bg-white rounded-xl shadow-lg p-6 transform transition-all duration-300 hover:scale-105 hover:shadow-xl">
    <Icon className="text-blue-600 w-12 h-12 mb-4" />
    <h3 className="text-xl font-bold mb-2 text-gray-800">{title}</h3>
    <p className="text-gray-600">{description}</p>
  </div>
);

const TechStack = ({ title, items }) => (
  <div className="bg-gray-100 rounded-xl p-6 shadow-inner">
    <h3 className="text-2xl font-bold mb-4 text-gray-800">{title}</h3>
    <ul className="space-y-2">
      {items.map((item, index) => (
        <li key={index} className="flex items-center text-gray-700">
          <div className="w-2 h-2 bg-blue-600 rounded-full mr-2"></div>
          {item}
        </li>
      ))}
    </ul>
  </div>
);

const TeamMember = ({ name, role, link }) => (
  <div className="text-center">
    <a href={link} target="_blank" rel="noopener noreferrer" className="block hover:opacity-80 transition-opacity">
      <div className="bg-white rounded-xl shadow-lg p-6 transition-all duration-300 hover:shadow-xl">
        <h3 className="text-2xl font-bold text-gray-800 mb-2">{name}</h3>
        <p className="text-lg text-gray-600">{role}</p>
      </div>
    </a>
  </div>
);

const LawnMowerSimulation = () => {
  const [mowerPosition, setMowerPosition] = useState({ x: 50, y: 50 });
  const [obstacles, setObstacles] = useState([
    { x: 30, y: 30 },
    { x: 70, y: 70 },
    { x: 20, y: 80 },
  ]);
  const [mowedAreas, setMowedAreas] = useState([]);
  const [isRunning, setIsRunning] = useState(false);
  const [batteryLevel, setBatteryLevel] = useState(100);
  const [simulationSpeed, setSimulationSpeed] = useState(1);

  const geofenceInset = 10;
  const gridSize = 5;

  const moveTowards = useCallback((current, target, speed) => {
    if (Math.abs(current - target) < speed) return target;
    return current + (target > current ? speed : -speed);
  }, []);

  const findNextTarget = useCallback(() => {
    for (let y = geofenceInset; y <= 100 - geofenceInset; y += gridSize) {
      for (let x = geofenceInset; x <= 100 - geofenceInset; x += gridSize) {
        if (!mowedAreas.some(area => 
          Math.abs(area.x - x) < gridSize && Math.abs(area.y - y) < gridSize
        )) {
          return { x, y };
        }
      }
    }
    return null; // All areas mowed
  }, [mowedAreas]);

  const checkCollision = useCallback((x, y) => {
    return obstacles.some(obs => 
      Math.abs(obs.x - x) < 5 && Math.abs(obs.y - y) < 5
    );
  }, [obstacles]);

  useEffect(() => {
    if (!isRunning) return;

    const moveMower = () => {
      setMowerPosition(prev => {
        const target = findNextTarget() || prev;
        let newX = moveTowards(prev.x, target.x, 1 * simulationSpeed);
        let newY = moveTowards(prev.y, target.y, 1 * simulationSpeed);

        if (checkCollision(newX, newY)) {
          const angle = Math.random() * 2 * Math.PI;
          newX = prev.x + Math.cos(angle) * 2 * simulationSpeed;
          newY = prev.y + Math.sin(angle) * 2 * simulationSpeed;
        }

        newX = Math.max(geofenceInset, Math.min(100 - geofenceInset, newX));
        newY = Math.max(geofenceInset, Math.min(100 - geofenceInset, newY));

        setMowedAreas(areas => [...areas, { x: newX, y: newY }]);
        setBatteryLevel(level => Math.max(0, level - 0.01 * simulationSpeed));

        return { x: newX, y: newY };
      });
    };

    const interval = setInterval(moveMower, 50);
    return () => clearInterval(interval);
  }, [isRunning, findNextTarget, checkCollision, simulationSpeed, moveTowards]);

  const handleToggleSimulation = () => {
    setIsRunning(prev => !prev);
  };

  const handleAddObstacle = (e) => {
    if (isRunning) return; // Prevent adding obstacles while simulation is running
    const rect = e.currentTarget.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * 100;
    const y = ((e.clientY - rect.top) / rect.height) * 100;
    setObstacles(prev => [...prev, { x, y }]);
  };

  const handleReset = () => {
    setIsRunning(false);
    setMowerPosition({ x: 50, y: 50 });
    setObstacles([
      { x: 30, y: 30 },
      { x: 70, y: 70 },
      { x: 20, y: 80 },
    ]);
    setMowedAreas([]);
    setBatteryLevel(100);
    setSimulationSpeed(1);
  };

  return (
    <div className="w-full overflow-hidden bg-gradient-to-br from-green-400 to-blue-500 rounded-3xl shadow-2xl">
      <div 
        className="relative w-full h-96 rounded-t-3xl overflow-hidden cursor-pointer" 
        onClick={handleAddObstacle}
      >
        {/* Geofence */}
        <div className="absolute inset-0 border-4 border-dashed border-red-500 rounded-3xl" style={{
          top: `${geofenceInset}%`,
          left: `${geofenceInset}%`,
          right: `${geofenceInset}%`,
          bottom: `${geofenceInset}%`,
        }}></div>

        {/* Mowed areas */}
        {mowedAreas.map((area, index) => (
          <div 
            key={index}
            className="absolute bg-green-300 rounded-full"
            style={{
              left: `${area.x}%`,
              top: `${area.y}%`,
              width: '4px',
              height: '4px',
            }}
          ></div>
        ))}

        {/* Lawn Mower */}
        <div 
          className="absolute w-10 h-10 bg-blue-600 rounded-full flex items-center justify-center shadow-lg transform -translate-x-1/2 -translate-y-1/2 transition-all duration-300 ease-in-out"
          style={{ left: `${mowerPosition.x}%`, top: `${mowerPosition.y}%` }}
        >
          <Navigation className="text-white" size={24} />
        </div>

        {/* Obstacles */}
        {obstacles.map((obs, index) => (
          <div 
            key={index}
            className="absolute w-8 h-8 bg-brown-500 rounded-lg flex items-center justify-center shadow-md transform -translate-x-1/2 -translate-y-1/2"
            style={{ left: `${obs.x}%`, top: `${obs.y}%` }}
          >
            <TreeDeciduous className="text-green-300" size={20} />
          </div>
        ))}

        {/* GPS Marker */}
        <div className="absolute top-4 right-4 text-blue-600 animate-pulse">
          <MapPin size={32} />
        </div>
      </div>

      {/* Controls */}
      <div className="bg-white rounded-b-3xl p-6">
        <div className="flex justify-between items-center mb-4">
          <div className="flex space-x-2">
            <button 
              onClick={handleToggleSimulation}
              className="bg-blue-600 text-white p-3 rounded-full hover:bg-blue-700 transition-colors duration-300"
            >
              {isRunning ? <Pause size={24} /> : <Play size={24} />}
            </button>
            <button
              onClick={handleReset}
              className="bg-gray-200 text-gray-700 p-3 rounded-full hover:bg-gray-300 transition-colors duration-300"
            >
              <RotateCcw size={24} />
            </button>
          </div>
          <div className="flex items-center space-x-2">
            <Battery size={24} />
            <div className="w-32 bg-gray-200 rounded-full h-3">
              <div 
                className="bg-green-500 h-3 rounded-full transition-all duration-300 ease-in-out"
                style={{ width: `${batteryLevel}%` }}
              ></div>
            </div>
            <span className="text-sm font-medium">{batteryLevel.toFixed(1)}%</span>
          </div>
        </div>
        <div className="flex items-center space-x-4">
          <Zap size={24} className="text-yellow-500" />
          <input
            type="range"
            min={0.1}
            max={5}
            step={0.1}
            value={simulationSpeed}
            onChange={(e) => setSimulationSpeed(parseFloat(e.target.value))}
            className="w-full"
          />
          <span className="text-sm font-medium">{simulationSpeed.toFixed(1)}x</span>
        </div>
      </div>
    </div>
  );
};

const ProjectPage = () => {
  const [activeSection, setActiveSection] = useState('overview');
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [showConnection, setShowConnection] = useState(false);
  const [showDashboard, setShowDashboard] = useState(false);

  useEffect(() => {
    const handleScroll = () => {
      const sections = ['overview', 'simulation', 'technology', 'team'];
      for (const section of sections) {
        const element = document.getElementById(section);
        if (element) {
          const rect = element.getBoundingClientRect();
          if (rect.top <= 100 && rect.bottom >= 100) {
            setActiveSection(section);
            break;
          }
        }
      }
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  const handleNavClick = useCallback((sectionId) => {
    setActiveSection(sectionId);
    setIsMenuOpen(false);
    const element = document.getElementById(sectionId);
    if (element) {
      element.scrollIntoView({ behavior: 'smooth' });
    }
  }, []);

  const handleConnectClick = () => {
    setShowConnection(true);
    setShowDashboard(false);
  };

  const handleDashboardClick = () => {
    setShowDashboard(true);
    setShowConnection(false);
  };

  // If either modal is shown, render it instead of the main content
  if (showConnection) {
    return (
      <div>
        <header className="fixed w-full z-50 bg-white shadow-md">
          <nav className="container mx-auto px-4 py-4">
            <div className="flex justify-between items-center">
              <h1 className="text-3xl font-extrabold text-blue-600 tracking-tight">AutoMow</h1>
              <button
                onClick={() => setShowConnection(false)}
                className="text-gray-600 hover:text-blue-600"
              >
                <X size={24} />
              </button>
            </div>
          </nav>
        </header>
        <RobotConnection />
      </div>
    );
  }

  if (showDashboard) {
    return (
      <div>
        <header className="fixed w-full z-50 bg-white shadow-md">
          <nav className="container mx-auto px-4 py-4">
            <div className="flex justify-between items-center">
              <h1 className="text-3xl font-extrabold text-blue-600 tracking-tight">AutoMow</h1>
              <button
                onClick={() => setShowDashboard(false)}
                className="text-gray-600 hover:text-blue-600"
              >
                <X size={24} />
              </button>
            </div>
          </nav>
        </header>
        <RobotDashboard />
      </div>
    );
  }

  return (
    <div className="min-h-screen font-sans bg-green-50">
      <header className="fixed w-full z-50 bg-white shadow-md">
        <nav className="container mx-auto px-4 py-4">
          <div className="flex justify-between items-center">
            <h1 className="text-3xl font-extrabold text-blue-600 tracking-tight">AutoMow</h1>
            <div className="hidden md:flex items-center space-x-4">
              <NavLink href="#overview" isActive={activeSection === 'overview'} onClick={() => handleNavClick('overview')}>Overview</NavLink>
              <NavLink href="#simulation" isActive={activeSection === 'simulation'} onClick={() => handleNavClick('simulation')}>Simulation</NavLink>
              <NavLink href="#technology" isActive={activeSection === 'technology'} onClick={() => handleNavClick('technology')}>Technology</NavLink>
              <NavLink href="#team" isActive={activeSection === 'team'} onClick={() => handleNavClick('team')}>Team</NavLink>
              <button
                onClick={handleConnectClick}
                className="flex items-center space-x-2 px-4 py-2 bg-blue-600 text-white rounded-full hover:bg-blue-700 transition-colors duration-300"
              >
                <Wifi size={20} />
                <span>Connect</span>
              </button>
              <button
                onClick={handleDashboardClick}
                className="flex items-center space-x-2 px-4 py-2 bg-green-600 text-white rounded-full hover:bg-green-700 transition-colors duration-300"
              >
                <Navigation size={20} />
                <span>Dashboard</span>
              </button>
            </div>
            <div className="md:hidden">
              <button onClick={() => setIsMenuOpen(!isMenuOpen)} className="text-gray-600 hover:text-blue-600">
                {isMenuOpen ? <X size={24} /> : <Menu size={24} />}
              </button>
            </div>
          </div>
        </nav>
        {isMenuOpen && (
          <div className="md:hidden bg-white py-4">
            <div className="container mx-auto px-4 flex flex-col space-y-4">
              <NavLink href="#overview" isActive={activeSection === 'overview'} onClick={() => handleNavClick('overview')}>Overview</NavLink>
              <NavLink href="#simulation" isActive={activeSection === 'simulation'} onClick={() => handleNavClick('simulation')}>Simulation</NavLink>
              <NavLink href="#technology" isActive={activeSection === 'technology'} onClick={() => handleNavClick('technology')}>Technology</NavLink>
              <NavLink href="#team" isActive={activeSection === 'team'} onClick={() => handleNavClick('team')}>Team</NavLink>
              <button
                onClick={handleConnectClick}
                className="flex items-center space-x-2 px-4 py-2 bg-blue-600 text-white rounded-full hover:bg-blue-700 transition-colors duration-300"
              >
                <Wifi size={20} />
                <span>Connect</span>
              </button>
              <button
                onClick={handleDashboardClick}
                className="flex items-center space-x-2 px-4 py-2 bg-green-600 text-white rounded-full hover:bg-green-700 transition-colors duration-300"
              >
                <Navigation size={20} />
                <span>Dashboard</span>
              </button>
            </div>
          </div>
        )}
      </header>
      
      <main className="pt-24">
        <Section id="overview" title="Revolutionizing Lawn Care" bgColor="bg-green-50">
          <div className="grid md:grid-cols-2 gap-12 items-center">
            <div>
              <p className="text-xl text-gray-600 mb-8 leading-relaxed">
                Welcome to the future of lawn maintenance. Our autonomous lawn mower harnesses the power of GPS and Visual SLAM technology to deliver precision, efficiency, and sustainability to your garden.
              </p>
              <div className="grid grid-cols-2 gap-6">
                <FeatureCard 
                  icon={MapPin} 
                  title="GPS Navigation" 
                  description="Precise positioning for accurate and efficient mowing patterns."
                />
                <FeatureCard 
                  icon={Navigation} 
                  title="Visual SLAM" 
                  description="Advanced mapping and obstacle avoidance for safe operation."
                />
                <FeatureCard 
                  icon={Zap} 
                  title="Energy Efficient" 
                  description="Smart power management for extended operation time."
                  />
                <FeatureCard 
                  icon={Info} 
                  title="Real-time Monitoring" 
                  description="Monitor and control your mower from anywhere via our web app."
                />
              </div>
            </div>
            <div className="relative">
              <div className="bg-gradient-to-br from-green-400 to-blue-500 rounded-3xl shadow-2xl p-8 text-white">
                <h3 className="text-4xl font-bold mb-4">Experience the Future</h3>
                <p className="text-xl mb-6">AutoMow brings cutting-edge technology to your lawn, ensuring a perfect cut every time.</p>
                <div className="grid grid-cols-2 gap-4">
                  <div className="bg-white bg-opacity-20 rounded-xl p-4">
                    <h4 className="font-bold mb-2">Smart Scheduling</h4>
                    <p>Set it and forget it with our intelligent mowing planner.</p>
                  </div>
                  <div className="bg-white bg-opacity-20 rounded-xl p-4">
                    <h4 className="font-bold mb-2">Easy Control</h4>
                    <p>Manage your mower with a secure web app from anywhere.</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
          <div className="mt-20 text-center">
            <a href="#simulation" className="inline-flex items-center text-blue-600 hover:text-blue-700">
              <span className="mr-2">Explore the Simulation</span>
              <ChevronDown size={20} />
            </a>
          </div>
        </Section>

        <Section id="simulation" title="Interactive Simulation" bgColor="bg-gradient-to-br from-blue-50 to-green-50">
          <div className="max-w-4xl mx-auto">
            <p className="text-xl text-gray-600 mb-8 text-center">
              Experience our autonomous lawn mower in action with this interactive simulation. Add obstacles, start/stop the mower, and observe its intelligent navigation.
            </p>
            <LawnMowerSimulation />
          </div>
        </Section>

        <Section id="technology" title="Cutting-Edge Technology" bgColor="bg-gradient-to-br from-blue-50 to-purple-50">
          <div className="grid md:grid-cols-2 gap-12">
            <TechStack 
              title="Hardware Components" 
              items={[
                "Raspberry Pi 4 for main processing",
                "Intel RealSense D435 camera for Visual SLAM",
                "High-precision GPS module",
                "Custom-designed chassis with efficient DC motors",
                "LiPo batteries with smart charging system"
              ]}
            />
            <TechStack 
              title="Software Stack" 
              items={[
                "Robot Operating System (ROS2) for system management",
                "OpenCV Library for image processing",
                "Custom path planning algorithms in Python",
                "React-based web application for remote control",
                "Node.js backend for data processing and storage"
              ]}
            />
          </div>
          <div className="mt-16">
            <h3 className="text-2xl font-bold mb-6 text-gray-800">Development Workflow</h3>
            <div className="bg-white rounded-2xl shadow-xl p-8">
              <div className="grid grid-cols-2 md:grid-cols-4 gap-8">
                {[
                  { title: "3D Modeling", description: "3D printing for the flawless final product", icon: "🖨️" },
                  { title: "Simulation", description: "Gazebo for advanced robotics testing", icon: "🎮" },
                  { title: "Board", description: "Raspberry Pi for powerful, compact computing", icon: "🍓" },
                  { title: "Model Design", description: "Fusion 360 for accurate design ", icon: "🔩" }
                ].map((step, index) => (
                  <div key={index} className="text-center">
                    <div className="text-4xl mb-4">{step.icon}</div>
                    <h4 className="text-xl font-bold mb-2 text-gray-800">{step.title}</h4>
                    <p className="text-gray-600">{step.description}</p>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </Section>

        <Section id="team" title="Meet Our Innovators" bgColor="bg-gradient-to-br from-purple-50 to-pink-50">
          <div className="text-center mb-12">
            <p className="text-xl text-gray-600 max-w-3xl mx-auto">
              Behind every great innovation is a team of passionate individuals. Meet the minds driving the future of autonomous lawn care.
            </p>
          </div>
          <div className="max-w-4xl mx-auto">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
              <TeamMember name="Pubesh Kumaar K S" role="CB.EN.U4CCE21048" link="#" />
              <TeamMember name="R Ezhil" role="CB.EN.U4CCE21049" link="https://ezhilravi.vercel.app" />
              <TeamMember name="A Srinithi" role="CB.EN.U4CCE21073" link="#" />
              <TeamMember name="Yazhini R" role="CB.EN.U4CCE21080" link="#" />
            </div>
          </div>
          <div className="mt-16 text-center">
            <h3 className="text-2xl font-bold mb-4 text-gray-800">Project Advisor</h3>
            <TeamMember 
              name="Mr. Peeyush K. P." 
              role="Assistant Professor, Department of Computer and Communication Engineering" 
              link="https://peeyushkp.my.canva.site/"
            />
          </div>
        </Section>
      </main>
    </div>
  );
};

export default ProjectPage;
