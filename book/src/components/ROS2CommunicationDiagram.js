import React, { useState } from 'react';
import styles from './ROS2CommunicationDiagram.module.css';

const ROS2CommunicationDiagram = () => {
  const [activeTab, setActiveTab] = useState('nodes'); // 'nodes', 'topics', 'services'

  const nodes = [
    { id: 'nav', name: 'Navigation Node', type: 'process', position: { x: 20, y: 40 } },
    { id: 'sensor', name: 'Laser Scanner', type: 'sensor', position: { x: 20, y: 20 } },
    { id: 'motor', name: 'Motor Controller', type: 'controller', position: { x: 20, y: 60 } },
    { id: 'gui', name: 'GUI Interface', type: 'interface', position: { x: 60, y: 40 } },
  ];

  const connections = [
    { from: 'sensor', to: 'nav', type: 'topic', name: '/scan' },
    { from: 'nav', to: 'motor', type: 'topic', name: '/cmd_vel' },
    { from: 'gui', to: 'nav', type: 'service', name: '/set_goal' },
  ];

  const renderNode = (node) => {
    let nodeClass = styles.node;
    if (node.type === 'sensor') nodeClass += ` ${styles.sensorNode}`;
    else if (node.type === 'controller') nodeClass += ` ${styles.controllerNode}`;
    else if (node.type === 'interface') nodeClass += ` ${styles.interfaceNode}`;
    else nodeClass += ` ${styles.processNode}`;

    return (
      <div
        key={node.id}
        className={nodeClass}
        style={{
          left: `${node.position.x}%`,
          top: `${node.position.y}%`,
        }}
      >
        <div className={styles.nodeName}>{node.name}</div>
        <div className={styles.nodeType}>{node.type}</div>
      </div>
    );
  };

  const renderConnection = (connection) => {
    // This is a simplified representation - in a real implementation,
    // you'd calculate the actual positions and draw SVG lines
    return (
      <div key={`${connection.from}-${connection.to}`} className={styles.connection}>
        <div className={styles.connectionLabel}>
          {connection.name} ({connection.type})
        </div>
      </div>
    );
  };

  return (
    <div className={styles.container}>
      <div className={styles.tabs}>
        <button
          className={activeTab === 'nodes' ? styles.activeTab : ''}
          onClick={() => setActiveTab('nodes')}
        >
          Nodes
        </button>
        <button
          className={activeTab === 'topics' ? styles.activeTab : ''}
          onClick={() => setActiveTab('topics')}
        >
          Topics
        </button>
        <button
          className={activeTab === 'services' ? styles.activeTab : ''}
          onClick={() => setActiveTab('services')}
        >
          Services
        </button>
      </div>

      <div className={styles.diagramContainer}>
        <h3>
          {activeTab === 'nodes' && 'ROS 2 Nodes Communication'}
          {activeTab === 'topics' && 'Topic-Based Communication (Publish/Subscribe)'}
          {activeTab === 'services' && 'Service-Based Communication (Request/Response)'}
        </h3>

        <div className={styles.diagram}>
          {nodes.map(renderNode)}
          {connections.map(renderConnection)}
        </div>

        <div className={styles.legend}>
          <div className={styles.legendItem}>
            <div className={`${styles.node} ${styles.processNode}`}></div>
            <span>Process Node</span>
          </div>
          <div className={styles.legendItem}>
            <div className={`${styles.node} ${styles.sensorNode}`}></div>
            <span>Sensor</span>
          </div>
          <div className={styles.legendItem}>
            <div className={`${styles.node} ${styles.controllerNode}`}></div>
            <span>Controller</span>
          </div>
          <div className={styles.legendItem}>
            <div className={`${styles.node} ${styles.interfaceNode}`}></div>
            <span>Interface</span>
          </div>
        </div>
      </div>

      <div className={styles.explanation}>
        {activeTab === 'nodes' && (
          <div>
            <h4>Nodes in ROS 2</h4>
            <p>
              A node is an individual process that performs computation. In ROS 2, nodes are the basic
              building blocks of a system. Each node can perform a specific task and communicate with
              other nodes to achieve complex behaviors.
            </p>
            <p>
              Nodes can be written in different programming languages (C++, Python, etc.) and can run
              on different machines in a distributed system.
            </p>
          </div>
        )}

        {activeTab === 'topics' && (
          <div>
            <h4>Topics - Publish/Subscribe Pattern</h4>
            <p>
              Topics provide a publish-subscribe communication pattern for continuous data streams.
              Topics enable asynchronous communication between nodes, where one or more nodes
              (publishers) send data to one or more other nodes (subscribers).
            </p>
            <p>
              Topics support many-to-many communication. Publishers don't know who subscribes to
              their data, and subscribers don't know who publishes the data they receive.
            </p>
          </div>
        )}

        {activeTab === 'services' && (
          <div>
            <h4>Services - Request/Response Pattern</h4>
            <p>
              Services provide request-response communication for specific tasks that require a reply.
              Services enable synchronous communication where a client sends a request and waits for
              a response from a server.
            </p>
            <p>
              Each service has exactly one server and can have multiple clients. Services are ideal
              for actions like changing robot state, executing specific commands, etc.
            </p>
          </div>
        )}
      </div>
    </div>
  );
};

export default ROS2CommunicationDiagram;