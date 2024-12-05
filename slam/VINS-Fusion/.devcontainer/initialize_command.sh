#!/usr/bin/env bash

# Define the paths
DEVCONTAINER_JSON=".devcontainer/devcontainer.json"
ENVIRONMENT_FILE=".devcontainer/.env"
ENVIRONMENT_VARIABLE="SELECTED_DIRECTORY"

echo "Set up a directory to be mounted inside the dev container."
echo "Note: Rebuild the container if the directory changes."

# Load the previous directory if it exists
if [[ -f "${ENVIRONMENT_FILE}" ]]; then
  source "${ENVIRONMENT_FILE}"

  if [[ ! -z "${!ENVIRONMENT_VARIABLE}" ]]; then
    echo "Previously selected directory: '${!ENVIRONMENT_VARIABLE}'"
    read -p "Would you like to select a different directory? [y/N]: " confirm
    [[ $confirm == y || $confirm == Y ]] || exit 0
  fi
fi

# Prompt the user for a new directory
read -p "Enter the full path of the directory to mount: " SELECTED_DIRECTORY

# Validate the input
if [[ -z "${SELECTED_DIRECTORY}" || ! -d "${SELECTED_DIRECTORY}" ]]; then
  echo "Error: Invalid directory. Aborting."
  exit 1
fi

# Ensure the directory path is absolute
SELECTED_DIRECTORY=$(realpath "${SELECTED_DIRECTORY}")

# Save the selected directory to the environment file
echo "SELECTED_DIRECTORY=${SELECTED_DIRECTORY}" > "${ENVIRONMENT_FILE}"
echo "Directory saved: ${SELECTED_DIRECTORY}"

# Update the devcontainer.json to replace the placeholder
if [[ -f "${DEVCONTAINER_JSON}" ]]; then
  sed -i "s|SELECTED_DIRECTORY|${SELECTED_DIRECTORY}|g" "${DEVCONTAINER_JSON}"
  echo "Updated devcontainer.json with directory: ${SELECTED_DIRECTORY}"
else
  echo "Error: ${DEVCONTAINER_JSON} not found. Aborting."
  exit 1
fi
