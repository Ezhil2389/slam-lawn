import React, { useState, useEffect, useCallback } from 'react';
import { useNavigate } from 'react-router-dom';
import { 
  MapPin, 
  Navigation, 
  Play, 
  Pause, 
  TreeDeciduous, 
  Info, 
  Battery, 
  Zap, 
  ChevronDown, 
  Menu, 
  X, 
  RotateCcw,
} from 'lucide-react';

const Section = ({ id, title, children, bgColor = 'bg-white' }) => (
  <section id={id} className={`min-h-screen ${bgColor} py-20 relative overflow-hidden`}>
    <div className="container mx-auto px-6 relative z-10">
      <h2 className="text-5xl font-extrabold mb-12 text-gray-800 tracking-tight">{title}</h2>
      {children}
    </div>
    <div className="absolute inset-0 bg-gradient-to-br from-transparent to-black opacity-5"></div>
  </section>
);

const NavLink = ({ href, children, isActive, onClick, isExternal = false }) => {
  const baseClasses = `text-lg font-medium transition-all duration-300 block ${
    isActive 
      ? 'text-blue-600 bg-blue-100 px-4 py-2 rounded-full' 
      : 'text-gray-600 hover:text-blue-600 hover:bg-blue-50 px-4 py-2 rounded-full'
  }`;

  if (isExternal) {
    return (
      <button
        onClick={onClick}
        className={baseClasses}
      >
        {children}
      </button>
    );
  }

  return (
    <a
      href={href}
      onClick={onClick}
      className={baseClasses}
    >
      {children}
    </a>
  );
};

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

// Your existing LawnMowerSimulation component remains the same

const ProjectPage = () => {
  const [activeSection, setActiveSection] = useState('overview');
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const navigate = useNavigate();

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
    navigate('/connect');
  };

  return (
    <div className="min-h-screen font-sans bg-green-50">
      <header className="fixed w-full z-50 bg-white shadow-md">
        <nav className="container mx-auto px-4 py-4">
          <div className="flex justify-between items-center">
            <h1 className="text-3xl font-extrabold text-blue-600 tracking-tight">AutoMow</h1>
            <div className="hidden md:flex space-x-4">
              <NavLink href="#overview" isActive={activeSection === 'overview'} onClick={() => handleNavClick('overview')}>Overview</NavLink>
              <NavLink href="#simulation" isActive={activeSection === 'simulation'} onClick={() => handleNavClick('simulation')}>Simulation</NavLink>
              <NavLink href="#technology" isActive={activeSection === 'technology'} onClick={() => handleNavClick('technology')}>Technology</NavLink>
              <NavLink href="#team" isActive={activeSection === 'team'} onClick={() => handleNavClick('team')}>Team</NavLink>
              <NavLink isExternal isActive={false} onClick={handleConnectClick}>
                <div className="flex items-center space-x-2">
                  
                  <span>Connect Robot</span>
                </div>
              </NavLink>
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
              <NavLink isExternal isActive={false} onClick={handleConnectClick}>
                <div className="flex items-center space-x-2">
                  
                  <span>Connect Robot</span>
                </div>
              </NavLink>
            </div>
          </div>
        )}
      </header>
      
      <main className="pt-24">
        {/* Your existing sections remain the same */}
        <Section id="overview" title="Revolutionizing Lawn Care" bgColor="bg-green-50">
          {/* Overview content remains the same */}
        </Section>

        <Section id="simulation" title="Interactive Simulation" bgColor="bg-gradient-to-br from-blue-50 to-green-50">
          {/* Simulation content remains the same */}
        </Section>

        <Section id="technology" title="Cutting-Edge Technology" bgColor="bg-gradient-to-br from-blue-50 to-purple-50">
          {/* Technology content remains the same */}
        </Section>

        <Section id="team" title="Meet Our Innovators" bgColor="bg-gradient-to-br from-purple-50 to-pink-50">
          {/* Team content remains the same */}
        </Section>
      </main>
    </div>
  );
};

export default ProjectPage;