import json
import os
from copy import deepcopy

# Base directory to save the subject files
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
base_dir = os.path.join(parent_dir, 'SubjectEventFiles')

# List of subjects
subjects = [f'P{i}' for i in range(7, 23)]

# List of events (modify this list according to your specific events)
CP_events = ['N_self', 'N_asst', 'Alt_self', 'Alt_asst', 'Alt2_self']
JA_Slow_events = ['N_self', 'Alt_self', 'FE_start', 'FE_end', 'PS_start', 'PS_end']
JA_Fast_events = ['N_self', 'Alt_self', 'FE_start', 'FE_end', 'PS_start', 'PS_end']
ROM_events = ['N_self', 'Alt_self']
ADL_events = ['N_self', 'Alt_self']

# Combine the events into one trials_dict
trials_dict = {'CP': CP_events, 'JA_Slow': JA_Slow_events, 'JA_Fast': JA_Fast_events, 'ROM': ROM_events, 'ADL': ADL_events}

# Create the base directory if it does not exist
if not os.path.exists(base_dir):
    os.makedirs(base_dir)

# Iterate over each subject
for subject in subjects:
    print(f"\nEntering data for {subject}.")

    # Alter the trials dict based on each subject
    subject_trials_dict = deepcopy(trials_dict)

    # ADL data missing for P6 and P7
    if subject in ['P6', 'P7']:
        if 'ADL' in subject_trials_dict:
            del subject_trials_dict['ADL']

    # For these subjects, use a pose from JA_Slow to represent the Alt2_self pose, since it wasn't performed in CP
    if subject in ['P1', 'P2', 'P3']:
        # Add Alt2_self to JA_Slow dict and JA_fast dict
        subject_trials_dict['CP'].remove('Alt2_self')
        subject_trials_dict['JA_Slow'].append('Alt2_self')

    # Dictionary to hold the events and times for the current subject
    subject_events_times = {}

    # Iterate through trials
    for trial, event_dict in subject_trials_dict.items():
        print(f"\n\tEntering data for {trial}.\n")

        events_times = {}

        # Iterate over each event
        for event in event_dict:

            # Prompt the user to enter the time for the current event
            time_input = input(f"\tEnter time for {event}: ")
            if time_input.lower() == 'none':
                time = None
            else:
                time = int(time_input)

            # Store the event and time in the subject's dictionary
            events_times[event] = time

        subject_events_times[trial] = events_times

    # Save the subject's data to a text file
    file_path = os.path.join(base_dir, f"{subject}_event_dict.txt")
    file_obj = open(file_path, 'w')
    file_obj.write(str(subject_events_times))
    file_obj.close()

    print(f"Data for {subject} saved.")

