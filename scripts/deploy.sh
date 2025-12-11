#!/bin/bash

# Deployment script for Physical AI & Humanoid Robotics book
# This script deploys the Docusaurus site to GitHub Pages

set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Deploy Script"
echo "==========================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if git is available
if ! command_exists git; then
    echo "❌ git is not installed. Please install git to proceed with deployment."
    exit 1
fi

# Check if we're in a git repository
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo "❌ Not in a git repository. Please initialize a git repository first."
    exit 1
fi

# Check if the book directory exists
if [ ! -d "book" ]; then
    echo "❌ book directory not found. Please make sure Docusaurus is set up correctly."
    exit 1
fi

# Get current branch
CURRENT_BRANCH=$(git branch --show-current)
echo "Current branch: $CURRENT_BRANCH"

# Build the Docusaurus site first
echo "Building Docusaurus site before deployment..."
cd book
if command_exists npm; then
    npm run build
elif command_exists yarn; then
    yarn build
else
    echo "❌ Neither npm nor yarn found. Cannot build Docusaurus site."
    exit 1
fi
cd ..

echo "✓ Docusaurus site built successfully"

# Check if docusaurus command is available for deployment
if command_exists docusaurus; then
    echo "Deploying using docusaurus command..."
    cd book
    docusaurus deploy
    cd ..
elif command_exists npm; then
    # Check if deploy script exists in package.json
    if grep -q "deploy" book/package.json; then
        echo "Deploying using npm script..."
        cd book
        npm run deploy
        cd ..
    else
        echo "Deploy script not found in package.json. Creating GitHub Pages deployment..."

        # Check if gh-pages is installed
        if ! npm list -g gh-pages > /dev/null 2>&1 && ! command_exists gh-pages; then
            echo "Installing gh-pages for deployment..."
            npm install -g gh-pages
        fi

        # Deploy to GitHub Pages
        echo "Deploying to GitHub Pages using gh-pages..."
        npx gh-pages -d book/build -r origin -b gh-pages
    fi
else
    echo "❌ Cannot deploy: no deployment method found."
    echo "Please install npm/node.js or ensure docusaurus command is available."
    exit 1
fi

# Additional deployment information
echo "==========================================="
echo "Deployment completed successfully!"
echo "==========================================="
echo ""
echo "Your site should now be live at:"
echo "https://$(git config user.name).github.io/humanoid-robotics-book/"
echo ""
echo "Note: It may take a few minutes for the changes to appear online."
echo ""
echo "If this is your first deployment, make sure to:"
echo "1. Go to your repository settings on GitHub"
echo "2. Navigate to Pages section"
echo "3. Select 'Deploy from a branch'"
echo "4. Choose 'gh-pages' branch and '/ (root)' folder"
echo "5. Save the settings"