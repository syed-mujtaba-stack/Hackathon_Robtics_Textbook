import React, { useState } from 'react';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
    content: string;
    onPersonalized: (personalizedContent: string) => void;
}

export default function PersonalizeButton({ content, onPersonalized }: PersonalizeButtonProps): React.JSX.Element {
    const [isLoading, setIsLoading] = useState(false);
    const [showModal, setShowModal] = useState(false);
    const [background, setBackground] = useState('');

    const handlePersonalize = async () => {
        setIsLoading(true);

        try {
            // TODO: Replace with actual AI API call
            // const response = await fetch('/api/personalize', {
            //   method: 'POST',
            //   headers: { 'Content-Type': 'application/json' },
            //   body: JSON.stringify({ content, background })
            // });
            // const data = await response.json();

            // Mock personalization for now
            const personalizedContent = `**Personalized for ${background} background:**\n\n${content}\n\n_This content has been adapted to your ${background} experience level._`;

            onPersonalized(personalizedContent);
            setShowModal(false);
        } catch (error) {
            console.error('Personalization failed:', error);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <>
            <button
                className={styles.personalizeBtn}
                onClick={() => setShowModal(true)}
            >
                ✨ Personalize Content
            </button>

            {showModal && (
                <div className={styles.modal}>
                    <div className={styles.modalContent}>
                        <h3>Personalize This Content</h3>
                        <p>Adapt this content to your background:</p>

                        <select
                            value={background}
                            onChange={(e) => setBackground(e.target.value)}
                            className={styles.select}
                        >
                            <option value="">Select your background</option>
                            <option value="beginner">Beginner (No robotics experience)</option>
                            <option value="student">Student (CS/Engineering major)</option>
                            <option value="professional">Professional (Industry experience)</option>
                            <option value="researcher">Researcher (Academic background)</option>
                        </select>

                        <div className={styles.actions}>
                            <button
                                onClick={handlePersonalize}
                                disabled={!background || isLoading}
                                className={styles.btnPrimary}
                            >
                                {isLoading ? 'Personalizing...' : 'Apply'}
                            </button>
                            <button
                                onClick={() => setShowModal(false)}
                                className={styles.btnSecondary}
                            >
                                Cancel
                            </button>
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}
