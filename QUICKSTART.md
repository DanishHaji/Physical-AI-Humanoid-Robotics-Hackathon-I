# 🚀 Quick Start - Deploy in 15 Minutes

The easiest way to deploy your Physical AI Textbook.

## Prerequisites (5 minutes)

### 1. Create Free Accounts
Open these links and sign up (all free, no credit card needed):

- **Groq** (AI): https://console.groq.com
- **Qdrant** (Vector DB): https://cloud.qdrant.io
- **Neon** (Database): https://neon.tech
- **Vercel** (Frontend): https://vercel.com/signup
- **Render** (Backend): https://render.com/register

### 2. Get API Keys
After signing up, get your credentials:

**Groq:**
1. Dashboard → API Keys → Create API Key
2. Copy the key

**Qdrant:**
1. Create new cluster (Free 1GB)
2. Copy cluster URL (e.g., `https://xxx.cloud.qdrant.io`)
3. Dashboard → API Keys → Create key → Copy it

**Neon:**
1. Create new project
2. Copy connection string from dashboard
3. Should look like: `postgresql://user:pass@host/dbname?sslmode=require`

---

## Automated Deployment (10 minutes)

### Option 1: Windows Users

```powershell
# Run the automated deployment script
.\deploy.ps1
```

### Option 2: Mac/Linux Users

```bash
# Make script executable
chmod +x deploy.sh

# Run the automated deployment script
./deploy.sh
```

### What the script does:
1. ✅ Checks if you have Node.js, Python, Git installed
2. ✅ Installs Vercel CLI
3. ✅ Asks for your API keys (paste them when prompted)
4. ✅ Creates `.env` file automatically
5. ✅ Sets up Python virtual environment
6. ✅ Installs all dependencies
7. ✅ Initializes databases (Neon + Qdrant)
8. ✅ Ingests textbook content to Qdrant
9. ✅ Deploys frontend to Vercel (opens browser for login)
10. ✅ Guides you through Render backend setup
11. ✅ Tests both deployments
12. ✅ Gives you your live URLs!

---

## Manual Deployment (If Script Fails)

See `DEPLOYMENT.md` for step-by-step manual instructions.

---

## Troubleshooting

**Script fails with "command not found"**
- Install missing software: Node.js, Python, or Git
- Restart terminal after installation

**Python virtual environment errors**
- Windows: Use PowerShell (not CMD)
- Mac/Linux: Make sure you have `python3-venv` installed

**Vercel login issues**
- The script will open a browser
- Login with your GitHub account
- Return to terminal after login

**Render deployment**
- Render requires manual setup (no CLI yet)
- Script will pause and guide you through browser setup
- Takes 5-10 minutes for first deploy

---

## After Deployment

Your app is now live! 🎉

1. **Frontend**: `https://your-app.vercel.app`
2. **Backend**: `https://your-api.onrender.com`

### Test it:
1. Visit your frontend URL
2. Click any chapter
3. Use chatbot (bottom-right corner)
4. Ask: "What is Physical AI?"

---

## Next Steps

- **Add custom domain**: See DEPLOYMENT.md
- **Monitor usage**: Check Vercel/Render dashboards
- **Update content**: Edit files in `docs/` folder
- **Redeploy**: Just `git push` - auto-deploys!

---

## Support

- **Issues**: https://github.com/DanishHaji/Physical-AI-Humanoid-Robotics-Hackathon-I/issues
- **Full guide**: See `DEPLOYMENT.md`
- **Project setup**: See `README.md`

---

**Total Time: ~15 minutes**
**Total Cost: $0/month** ✨
