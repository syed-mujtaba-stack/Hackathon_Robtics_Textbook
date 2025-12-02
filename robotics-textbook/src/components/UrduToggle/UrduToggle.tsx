import React, { useState } from 'react';
import styles from './UrduToggle.module.css';

interface UrduToggleProps {
    onToggle: (isUrdu: boolean) => void;
}

export default function UrduToggle({ onToggle }: UrduToggleProps): React.JSX.Element {
    const [isUrdu, setIsUrdu] = useState(false);

    const handleToggle = () => {
        const newValue = !isUrdu;
        setIsUrdu(newValue);
        onToggle(newValue);
    };

    return (
        <div className={styles.toggleContainer}>
            <span className={`${styles.label} ${!isUrdu ? styles.active : ''}`}>
                English
            </span>
            <button
                className={`${styles.toggle} ${isUrdu ? styles.toggled : ''}`}
                onClick={handleToggle}
                aria-label="Toggle language"
            >
                <span className={styles.slider} />
            </button>
            <span className={`${styles.label} ${isUrdu ? styles.active : ''}`}>
                اردو
            </span>
        </div>
    );
}
